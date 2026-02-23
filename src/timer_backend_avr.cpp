// timer_backend_avr.cpp - ATmega2560 Timer3/4 CTC step generation
//
// Timer3: Master ISR for dominant-axis stepping + Bresenham minor-axis
// Timer3 CompB: Drives STEP pins LOW after minimum pulse width
// Timer4: Reserved for pure-minor-axis moves (future use)

#ifndef UNIT_TEST

#include "timer_backend.h"
#include "coordinated_stepper.h"
#include "bresenham_sync.h"
#include <Arduino.h>
#include <avr/interrupt.h>

// ===== Shared state between main loop and ISR =====

// LUT double-buffer: ISR reads one half while main loop fills the other.
// The ISR wraps s_lutIndex circularly through 0..LUT_TOTAL_SIZE-1.
// When it crosses the midpoint or wraps around, it signals s_needRefill
// so the main loop can fill the inactive half with the next profile steps.
static uint16_t s_lut[LUT_TOTAL_SIZE];
static volatile int16_t s_lutIndex = 0;       // circular read position (0 to LUT_TOTAL_SIZE-1)
static volatile bool s_firstHalfActive = true; // which half the ISR is currently reading
static volatile bool s_needRefill = false;     // main loop should refill inactive half
static int32_t s_nextComputeStep = 0;          // next profile step to compute into LUT (main loop only)
static volatile bool s_lutExhausted = false;   // true if no more profile steps to fill

// Move state accessed from ISR
static volatile bool s_moveActive = false;
static volatile int32_t s_stepsRemaining = 0;
static volatile int32_t s_isrFireCount = 0;  // diagnostic: total ISR fires

// Axis port registers cached for ISR (set at move start)
static volatile uint8_t* s_domStepPort = nullptr;
static uint8_t s_domStepMask = 0;
static volatile uint8_t* s_minStepPort = nullptr;
static uint8_t s_minStepMask = 0;
static volatile bool s_minStepPendingLow = false;

// Bresenham state for ISR
static volatile int32_t s_bresError = 0;
static volatile int32_t s_bresDominant = 0;
static volatile int32_t s_bresMinor = 0;

// Dominant axis position tracking
static StepperAxis* s_domAxis = nullptr;
static StepperAxis* s_minAxis = nullptr;

// Prescaler state
static volatile bool s_usingPrescaler8 = false;

// ===== Timer Configuration =====

void timerBackendInit() {
    noInterrupts();

    // Timer3: CTC mode (WGM = 0100), prescaler = 1 (CS30)
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3  = 0;
    OCR3A  = 65535;               // Start at slowest rate (motor stopped)
    TCCR3B |= (1 << WGM32);      // CTC mode
    TCCR3B |= (1 << CS30);       // Prescaler = 1 (16 MHz tick)
    // Don't enable interrupt yet — enabled when a move starts
    TIMSK3 &= ~(1 << OCIE3A);
    TIMSK3 &= ~(1 << OCIE3B);

    // Timer4: CTC mode, prescaler = 1 (reserved for future use)
    TCCR4A = 0;
    TCCR4B = 0;
    TCNT4  = 0;
    OCR4A  = 65535;
    TCCR4B |= (1 << WGM42);
    TCCR4B |= (1 << CS40);
    TIMSK4 &= ~(1 << OCIE4A);

    s_moveActive = false;

    interrupts();
}

void timerBackendStartMove(CoordinatedStepper& stepper) {
    if (stepper.isComplete()) {
        Serial.println(F("[TMR] startMove: stepper already complete, skipping"));
        return;
    }

    noInterrupts();

    // Stop any active move
    TIMSK3 &= ~(1 << OCIE3A);
    TIMSK3 &= ~(1 << OCIE3B);
    s_moveActive = false;

    const SCurveProfile& prof = stepper.profile();
    s_stepsRemaining = prof.totalSteps();

    if (s_stepsRemaining <= 0) {
        Serial.println(F("[TMR] startMove: 0 steps in profile, skipping"));
        interrupts();
        return;
    }

    // Cache port registers for dominant and minor axes
    bool thetaDom = stepper.thetaAxis().stepsRemaining >= stepper.radiusAxis().stepsRemaining;

    if (thetaDom) {
        s_domAxis = &stepper.thetaAxis();
        s_minAxis = &stepper.radiusAxis();
    } else {
        s_domAxis = &stepper.radiusAxis();
        s_minAxis = &stepper.thetaAxis();
    }

    s_domStepPort = s_domAxis->stepPort;
    s_domStepMask = s_domAxis->stepMask;
    s_minStepPort = s_minAxis->stepPort;
    s_minStepMask = s_minAxis->stepMask;
    s_minStepPendingLow = false;

    // Copy Bresenham state
    int32_t absDom = (s_domAxis->stepsRemaining >= 0) ? s_domAxis->stepsRemaining : -s_domAxis->stepsRemaining;
    int32_t absMin = (s_minAxis->stepsRemaining >= 0) ? s_minAxis->stepsRemaining : -s_minAxis->stepsRemaining;
    s_bresDominant = absDom;
    s_bresMinor = absMin;
    s_bresError = absDom / 2;

    // Precompute first LUT block
    int32_t count = prof.precomputeLUT(s_lut, LUT_TOTAL_SIZE, F_CPU);
    s_nextComputeStep = count;  // next profile step to compute on refill
    s_lutExhausted = (count >= prof.totalSteps());
    s_lutIndex = 1;  // s_lut[0] is used as the initial OCR3A; ISR reads from [1] onward
    s_firstHalfActive = true;
    s_needRefill = false;

    // Set initial interval
    uint16_t firstInterval = (count > 0) ? s_lut[0] : 65535;

    // Check if we need prescaler 8 for slow start
    if (firstInterval >= 65535) {
        // Switch to prescaler 8 for slow speeds
        TCCR3B = (1 << WGM32) | (1 << CS31);  // prescaler = 8
        s_usingPrescaler8 = true;
        firstInterval = firstInterval >> 3;  // Adjust for prescaler
        if (firstInterval < 1) firstInterval = 1;
    } else {
        TCCR3B = (1 << WGM32) | (1 << CS30);  // prescaler = 1
        s_usingPrescaler8 = false;
    }

    OCR3A = firstInterval;
    TCNT3 = 0;

    s_isrFireCount = 0;
    s_moveActive = true;

    // Enable Timer3 CompA interrupt to start stepping
    TIMSK3 |= (1 << OCIE3A);

    interrupts();

    Serial.print(F("[TMR] ISR armed: steps="));
    Serial.print(s_stepsRemaining);
    Serial.print(F(" lutEntries="));
    Serial.print(s_nextComputeStep);
    Serial.print(F(" firstInterval="));
    Serial.print(s_lut[0]);
    Serial.print(F(" domPort=0x"));
    Serial.print((unsigned long)(uintptr_t)s_domStepPort, 16);
    Serial.print(F(" mask=0x"));
    Serial.print(s_domStepMask, 16);
    Serial.print(F(" prescaler="));
    Serial.println(s_usingPrescaler8 ? F("8") : F("1"));
}

void timerBackendStop() {
    noInterrupts();
    TIMSK3 &= ~(1 << OCIE3A);
    TIMSK3 &= ~(1 << OCIE3B);
    TIMSK4 &= ~(1 << OCIE4A);
    s_moveActive = false;
    interrupts();
}

bool timerBackendRefillLUT(CoordinatedStepper& stepper) {
    if (!s_moveActive) return false;
    if (!s_needRefill) return true;

    const SCurveProfile& prof = stepper.profile();

    // How many profile steps remain to be computed?
    int32_t remaining = prof.totalSteps() - s_nextComputeStep;
    int32_t toCompute = (remaining < LUT_HALF_SIZE) ? remaining : LUT_HALF_SIZE;

    if (toCompute <= 0) {
        // No more steps to compute — mark exhausted so ISR stops requesting refills
        noInterrupts();
        s_lutExhausted = true;
        s_needRefill = false;
        interrupts();
        return true;
    }

    // Determine which half to refill (opposite of what ISR is reading)
    int16_t refillStart;
    if (s_firstHalfActive) {
        refillStart = LUT_HALF_SIZE;  // ISR reading first half → refill second
    } else {
        refillStart = 0;              // ISR reading second half → refill first
    }

    // Compute intervals for this block using tracked profile step position
    prof.computeIntervalBlock(s_nextComputeStep, &s_lut[refillStart], toCompute, F_CPU);
    s_nextComputeStep += toCompute;

    noInterrupts();
    if (s_nextComputeStep >= prof.totalSteps()) {
        s_lutExhausted = true;
    }
    s_needRefill = false;
    interrupts();

    return true;
}

bool timerBackendIsComplete() {
    return !s_moveActive;
}

int32_t timerBackendISRCount() {
    noInterrupts();
    int32_t c = s_isrFireCount;
    interrupts();
    return c;
}

int32_t timerBackendStepsRemaining() {
    noInterrupts();
    int32_t r = s_stepsRemaining;
    interrupts();
    return r;
}

// ===== Timer3 CompA ISR: Master step ISR =====
ISR(TIMER3_COMPA_vect) {
    if (!s_moveActive || s_stepsRemaining <= 0) {
        // Move complete — disable interrupt
        TIMSK3 &= ~(1 << OCIE3A);
        s_moveActive = false;
        return;
    }

    s_isrFireCount++;

    // 1. Fire dominant axis STEP HIGH (single-cycle PORT write)
    *s_domStepPort |= s_domStepMask;
    s_domAxis->advancePosition();
    s_stepsRemaining--;

    // 2. Bresenham: check if minor axis also steps
    s_bresError += s_bresMinor;
    if (s_bresError >= s_bresDominant) {
        *s_minStepPort |= s_minStepMask;  // Minor STEP HIGH
        s_minAxis->advancePosition();
        s_bresError -= s_bresDominant;
        s_minStepPendingLow = true;
    }

    // 3. Read next interval from circular LUT
    uint16_t nextInterval = s_lut[s_lutIndex];
    s_lutIndex++;

    // Check if we've crossed the half-boundary — signal refill of the other half
    if (s_lutIndex == LUT_HALF_SIZE && s_firstHalfActive) {
        s_firstHalfActive = false;
        if (!s_lutExhausted) s_needRefill = true;
    } else if (s_lutIndex == LUT_TOTAL_SIZE) {
        s_lutIndex = 0;
        s_firstHalfActive = true;
        if (!s_lutExhausted) s_needRefill = true;
    }

    // 4. Handle prescaler switching.
    // LUT values are always computed for 16 MHz (prescaler=1).
    // When using prescaler=8, the timer ticks at 2 MHz, so we must
    // divide the LUT value by 8 before writing to OCR3A.
    if (s_usingPrescaler8) {
        if (nextInterval < (65535 / 8)) {
            // Speed high enough to use prescaler=1 — switch back
            TCCR3B = (1 << WGM32) | (1 << CS30);
            s_usingPrescaler8 = false;
            // nextInterval is already correct for 16 MHz
        } else {
            // Stay on prescaler=8 — convert 16 MHz value to 2 MHz
            nextInterval >>= 3;
            if (nextInterval < 1) nextInterval = 1;
        }
    } else {
        if (nextInterval >= 65535) {
            // Speed too low for prescaler=1 — switch to prescaler=8
            TCCR3B = (1 << WGM32) | (1 << CS31);
            s_usingPrescaler8 = true;
            nextInterval >>= 3;
            if (nextInterval < 1) nextInterval = 1;
        }
    }

    OCR3A = nextInterval;

    // 5. Schedule STEP LOW via CompB after minimum pulse width
    OCR3B = TCNT3 + PULSE_TICKS;
    TIMSK3 |= (1 << OCIE3B);

    // 6. Check if move is complete
    if (s_stepsRemaining <= 0) {
        s_moveActive = false;
        TIMSK3 &= ~(1 << OCIE3A);
    }
}

// ===== Timer3 CompB ISR: Drive STEP pins LOW =====
ISR(TIMER3_COMPB_vect) {
    // Drive dominant STEP LOW
    *s_domStepPort &= ~s_domStepMask;

    // Drive minor STEP LOW if it was pulsed
    if (s_minStepPendingLow) {
        *s_minStepPort &= ~s_minStepMask;
        s_minStepPendingLow = false;
    }

    // Disable CompB until next step
    TIMSK3 &= ~(1 << OCIE3B);
}

#endif // UNIT_TEST
