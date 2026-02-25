// timer_backend_avr.cpp - ATmega2560 Timer3/4 CTC step generation
//
// Timer3: Master ISR for dominant-axis stepping + Bresenham minor-axis
// Timer3 CompB: Drives STEP pins LOW after minimum pulse width
// Timer4: Reserved for pure-minor-axis moves (future use)
//
// Two execution modes:
//   Single-move (s_multiBlockMode == false):
//     Double-buffered LUT[0..1023], circular read. Used by homing path.
//   Multi-block (s_multiBlockMode == true):
//     Linear LUT[0..AVR_MAX_PRECOMPUTE_STEPS-1] with BlockMeta array.
//     Seamless ISR block transitions via load_next_block().

#ifndef UNIT_TEST

#include "timer_backend.h"
#include "coordinated_stepper.h"
#include "bresenham_sync.h"
#include <Arduino.h>
#include <avr/interrupt.h>

// ===== LUT: shared between single-move and multi-block modes =====
// Single-move uses [0..LUT_TOTAL_SIZE-1] (1024 entries, double-buffered).
// Multi-block uses [0..AVR_MAX_PRECOMPUTE_STEPS-1] (3000 entries, linear).
static uint16_t s_lut[AVR_MAX_PRECOMPUTE_STEPS];

// ===== Mode flag =====
static volatile bool s_multiBlockMode = false;

// ===== Single-move mode state =====
static volatile int16_t s_lutIndex = 0;
static volatile bool s_firstHalfActive = true;
static volatile bool s_needRefill = false;
static int32_t s_nextComputeStep = 0;
static volatile bool s_lutExhausted = false;

// ===== Multi-block mode state =====
static BlockMeta s_blockMeta[PLANNER_BUFFER_CAPACITY];
static volatile uint8_t s_blockCount = 0;
static volatile uint8_t s_currentBlockIdx = 0;
static volatile uint16_t s_mbLutIndex = 0;      // linear read position
static volatile uint16_t s_mbBlockEnd = 0;       // lut_start + lut_count for current block (deprecated, kept for compat)
static volatile uint16_t s_mbTotalLutSize = 0;   // total entries in LUT

// ===== Cruise phase compression support =====
static volatile bool     s_cruiseMode = false;   // ISR in cruise phase repeat mode
static volatile uint32_t s_cruiseRemaining = 0;  // steps left in cruise phase
static volatile uint16_t s_cruiseInterval = 0;   // the interval to repeat during cruise
static volatile int32_t  s_mbStepsInBlock = 0;   // step count for current block (replaces LUT boundary check)

// ===== Constant-acceleration sentinel support =====
static volatile bool     s_accelMode = false;      // ISR in const-accel/decel computation mode
static volatile uint32_t s_accelRemaining = 0;     // steps left in accel sentinel
static volatile uint32_t s_accelVelocity = 0;      // Q16.16 velocity (steps/sec * 65536)
static volatile int32_t  s_accelRate = 0;           // Q16.16 acceleration per step (signed)

// ===== Common move state =====
static volatile bool s_moveActive = false;
static volatile int32_t s_stepsRemaining = 0;
static volatile int32_t s_isrFireCount = 0;

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

// Axis pointers for multi-block direction switching
static StepperAxis* s_thetaAxis = nullptr;
static StepperAxis* s_radiusAxis = nullptr;

// Prescaler state
static volatile bool s_usingPrescaler8 = false;

// Abort / stall flags
static volatile bool s_abortRequested = false;
static volatile bool s_stalled = false;

// Forced deceleration state
static volatile bool s_forcedDecel = false;
static volatile uint16_t s_forcedDecelInterval = 0;
static const uint16_t FORCED_DECEL_INCREMENT = 50;        // ticks added per step
static const uint16_t FORCED_DECEL_STOP_INTERVAL = 60000; // stop threshold

// ===== Timer Configuration =====

void timerBackendInit() {
    noInterrupts();

    // Timer3: CTC mode (WGM = 0100), prescaler = 1 (CS30)
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3  = 0;
    OCR3A  = 65535;
    TCCR3B |= (1 << WGM32);
    TCCR3B |= (1 << CS30);
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
    s_multiBlockMode = false;
    s_abortRequested = false;
    s_stalled = false;
    s_forcedDecel = false;

    interrupts();
}

// ===== Single-move mode (backward compatible, for homing) =====

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
    s_multiBlockMode = false;
    s_abortRequested = false;
    s_stalled = false;
    s_forcedDecel = false;

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

    // Precompute first LUT block (uses only first LUT_TOTAL_SIZE entries)
    int32_t count = prof.precomputeLUT(s_lut, LUT_TOTAL_SIZE, F_CPU);
    s_nextComputeStep = count;
    s_lutExhausted = (count >= prof.totalSteps());
    s_lutIndex = 1;  // s_lut[0] is used as the initial OCR3A
    s_firstHalfActive = true;
    s_needRefill = false;

    // Set initial interval
    uint16_t firstInterval = (count > 0) ? s_lut[0] : 65535;

    // Check if we need prescaler 8 for slow start
    if (firstInterval >= 65535) {
        TCCR3B = (1 << WGM32) | (1 << CS31);  // prescaler = 8
        s_usingPrescaler8 = true;
        firstInterval = firstInterval >> 3;
        if (firstInterval < 1) firstInterval = 1;
    } else {
        TCCR3B = (1 << WGM32) | (1 << CS30);  // prescaler = 1
        s_usingPrescaler8 = false;
    }

    OCR3A = firstInterval;
    TCNT3 = 0;

    s_isrFireCount = 0;
    s_moveActive = true;

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

bool timerBackendRefillLUT(CoordinatedStepper& stepper) {
    if (!s_moveActive) return false;
    if (s_multiBlockMode) return true;  // multi-block doesn't use refill
    if (!s_needRefill) return true;

    const SCurveProfile& prof = stepper.profile();

    int32_t remaining = prof.totalSteps() - s_nextComputeStep;
    int32_t toCompute = (remaining < LUT_HALF_SIZE) ? remaining : LUT_HALF_SIZE;

    if (toCompute <= 0) {
        noInterrupts();
        s_lutExhausted = true;
        s_needRefill = false;
        interrupts();
        return true;
    }

    int16_t refillStart;
    if (s_firstHalfActive) {
        refillStart = LUT_HALF_SIZE;
    } else {
        refillStart = 0;
    }

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

// ===== Multi-block mode =====

// Inline helper: load the next block's Bresenham/direction state.
// Called from within the ISR when the current block's LUT entries are exhausted.
// Returns true if a block was loaded, false if all blocks are done.
static inline bool load_next_block() __attribute__((always_inline));
static inline bool load_next_block() {
    s_currentBlockIdx++;
    if (s_currentBlockIdx >= s_blockCount) {
        return false;
    }

    const BlockMeta& meta = s_blockMeta[s_currentBlockIdx];

    // Update Bresenham state
    s_bresDominant = meta.bres_dom;
    s_bresMinor = meta.bres_minor;
    s_bresError = meta.bres_dom / 2;

    // Update dominant/minor axis assignments
    if (meta.theta_dominant) {
        s_domAxis = s_thetaAxis;
        s_minAxis = s_radiusAxis;
    } else {
        s_domAxis = s_radiusAxis;
        s_minAxis = s_thetaAxis;
    }

    // Update port register caches
    s_domStepPort = s_domAxis->stepPort;
    s_domStepMask = s_domAxis->stepMask;
    s_minStepPort = s_minAxis->stepPort;
    s_minStepMask = s_minAxis->stepMask;

    // Update directions if changed
    if (s_thetaAxis->direction() != meta.dir_theta) {
        s_thetaAxis->setDirection(meta.dir_theta);
    }
    if (s_radiusAxis->direction() != meta.dir_radius) {
        s_radiusAxis->setDirection(meta.dir_radius);
    }

    // Update LUT read position and block boundary
    s_mbLutIndex = meta.lut_start;
    s_mbBlockEnd = meta.lut_start + meta.lut_count;

    // Update step counting for block boundary detection (replaces LUT boundary check)
    s_mbStepsInBlock = meta.bres_dom;

    // Reset compression modes for new block
    s_cruiseMode = false;
    s_cruiseRemaining = 0;
    s_accelMode = false;
    s_accelRemaining = 0;

    // stepsRemaining tracks total steps (for diagnostics)
    s_stepsRemaining = meta.bres_dom;

    return true;
}

uint16_t* timerBackendGetLutBuffer() {
    return s_lut;
}

void timerBackendLoadPlan(const BlockMeta* metas, uint8_t blockCount,
                          uint16_t lutSize,
                          StepperAxis* thetaAxis, StepperAxis* radiusAxis) {
    if (blockCount == 0 || lutSize == 0) return;

    noInterrupts();

    // Stop any active move
    TIMSK3 &= ~(1 << OCIE3A);
    TIMSK3 &= ~(1 << OCIE3B);
    s_moveActive = false;

    // Copy block metadata
    uint8_t count = (blockCount > PLANNER_BUFFER_CAPACITY) ? PLANNER_BUFFER_CAPACITY : blockCount;
    for (uint8_t i = 0; i < count; i++) {
        s_blockMeta[i] = metas[i];
    }
    s_blockCount = count;

    // LUT data already written directly into s_lut by the caller
    s_mbTotalLutSize = (lutSize > AVR_MAX_PRECOMPUTE_STEPS) ? AVR_MAX_PRECOMPUTE_STEPS : lutSize;

    // Store axis pointers for multi-block direction switching
    s_thetaAxis = thetaAxis;
    s_radiusAxis = radiusAxis;

    // Set up first block
    s_currentBlockIdx = 0;
    const BlockMeta& first = s_blockMeta[0];

    // Set directions
    thetaAxis->setDirection(first.dir_theta);
    radiusAxis->setDirection(first.dir_radius);

    // Set dominant/minor for first block
    if (first.theta_dominant) {
        s_domAxis = thetaAxis;
        s_minAxis = radiusAxis;
    } else {
        s_domAxis = radiusAxis;
        s_minAxis = thetaAxis;
    }

    s_domStepPort = s_domAxis->stepPort;
    s_domStepMask = s_domAxis->stepMask;
    s_minStepPort = s_minAxis->stepPort;
    s_minStepMask = s_minAxis->stepMask;
    s_minStepPendingLow = false;

    // Bresenham for first block
    s_bresDominant = first.bres_dom;
    s_bresMinor = first.bres_minor;
    s_bresError = first.bres_dom / 2;

    // LUT read position and step counting
    s_mbLutIndex = first.lut_start + 1;  // skip first entry used as initial OCR3A
    s_mbBlockEnd = first.lut_start + first.lut_count;
    s_mbStepsInBlock = first.bres_dom;  // step countdown for block boundary
    s_stepsRemaining = first.bres_dom;  // total step count for this block

    // Initialize compression state
    s_cruiseMode = false;
    s_cruiseRemaining = 0;
    s_accelMode = false;
    s_accelRemaining = 0;

    // Mode flags
    s_multiBlockMode = true;
    s_abortRequested = false;
    s_stalled = false;
    s_forcedDecel = false;

    // Set initial interval from first LUT entry
    uint16_t firstInterval = s_lut[first.lut_start];

    // Prescaler handling
    if (firstInterval >= 65535) {
        TCCR3B = (1 << WGM32) | (1 << CS31);  // prescaler = 8
        s_usingPrescaler8 = true;
        firstInterval = firstInterval >> 3;
        if (firstInterval < 1) firstInterval = 1;
    } else {
        TCCR3B = (1 << WGM32) | (1 << CS30);  // prescaler = 1
        s_usingPrescaler8 = false;
    }

    OCR3A = firstInterval;
    TCNT3 = 0;

    s_isrFireCount = 0;
    s_moveActive = true;

    TIMSK3 |= (1 << OCIE3A);

    interrupts();

    Serial.print(F("[TMR] Plan loaded: blocks="));
    Serial.print(s_blockCount);
    Serial.print(F(" lutSize="));
    Serial.print(s_mbTotalLutSize);
    Serial.print(F(" firstInterval="));
    Serial.println(s_lut[first.lut_start]);
}

// ===== Common API =====

void timerBackendStop() {
    noInterrupts();
    TIMSK3 &= ~(1 << OCIE3A);
    TIMSK3 &= ~(1 << OCIE3B);
    TIMSK4 &= ~(1 << OCIE4A);
    s_moveActive = false;
    s_forcedDecel = false;
    interrupts();
}

bool timerBackendIsComplete() {
    return !s_moveActive;
}

void timerBackendAbort() {
    s_abortRequested = true;
}

bool timerBackendIsStalled() {
    return s_stalled;
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
    if (!s_moveActive) {
        TIMSK3 &= ~(1 << OCIE3A);
        return;
    }

    // Check abort request — enter forced deceleration
    if (s_abortRequested && !s_forcedDecel) {
        s_forcedDecel = true;
        s_forcedDecelInterval = OCR3A;
        s_abortRequested = false;
    }

    // Forced deceleration path: ramp down to stop regardless of LUT/blocks
    if (s_forcedDecel) {
        if (s_forcedDecelInterval >= FORCED_DECEL_STOP_INTERVAL) {
            s_moveActive = false;
            s_stalled = true;
            TIMSK3 &= ~(1 << OCIE3A);
            return;
        }

        // Still stepping during decel
        *s_domStepPort |= s_domStepMask;
        s_domAxis->advancePosition();
        s_stepsRemaining--;
        s_isrFireCount++;

        // Bresenham minor
        s_bresError += s_bresMinor;
        if (s_bresError >= s_bresDominant) {
            *s_minStepPort |= s_minStepMask;
            s_minAxis->advancePosition();
            s_bresError -= s_bresDominant;
            s_minStepPendingLow = true;
        }

        // Ramp interval up (slow down)
        s_forcedDecelInterval += FORCED_DECEL_INCREMENT;
        OCR3A = s_forcedDecelInterval;

        // Schedule STEP LOW
        OCR3B = TCNT3 + PULSE_TICKS;
        TIMSK3 |= (1 << OCIE3B);
        return;
    }

    // ===== Normal stepping path =====
    s_isrFireCount++;

    // 1. Fire dominant axis STEP HIGH
    *s_domStepPort |= s_domStepMask;
    s_domAxis->advancePosition();
    s_stepsRemaining--;

    // 2. Bresenham: check if minor axis also steps
    s_bresError += s_bresMinor;
    if (s_bresError >= s_bresDominant) {
        *s_minStepPort |= s_minStepMask;
        s_minAxis->advancePosition();
        s_bresError -= s_bresDominant;
        s_minStepPendingLow = true;
    }

    // 3. Get next interval (mode-dependent)
    uint16_t nextInterval;

    if (s_multiBlockMode) {
        // Multi-block with cruise compression support
        // Step 1: Check block boundary (step-count-based)
        --s_mbStepsInBlock;
        if (s_mbStepsInBlock <= 0) {
            // Last step of this block just fired — load next block
            if (!load_next_block()) {
                // All blocks done
                s_moveActive = false;
                TIMSK3 &= ~(1 << OCIE3A);
                OCR3B = TCNT3 + PULSE_TICKS;
                TIMSK3 |= (1 << OCIE3B);
                return;
            }
            // Fall through to read first interval of new block
        }

        // Step 2: Get next interval from compressed LUT
        if (s_accelMode) {
            // In constant-accel phase: compute interval from velocity
            s_accelVelocity = (uint32_t)((int32_t)s_accelVelocity + s_accelRate);
            uint16_t vel16 = (uint16_t)(s_accelVelocity >> 16);
            nextInterval = (vel16 > 0) ? (uint16_t)((uint32_t)F_CPU / vel16) : 65535;
            if (--s_accelRemaining == 0) {
                s_accelMode = false;
            }
        } else if (s_cruiseMode) {
            // In cruise phase: repeat the cruise interval
            nextInterval = s_cruiseInterval;
            if (--s_cruiseRemaining == 0) {
                s_cruiseMode = false;
            }
        } else {
            // Read from LUT, check for sentinels
            nextInterval = s_lut[s_mbLutIndex];
            s_mbLutIndex++;

            if (nextInterval == 0) {
                // Cruise sentinel: [0, interval, count_hi, count_lo]
                s_cruiseInterval = s_lut[s_mbLutIndex++];
                uint16_t countHi = s_lut[s_mbLutIndex++];
                uint16_t countLo = s_lut[s_mbLutIndex++];
                uint32_t cruiseCount = ((uint32_t)countHi << 16) | countLo;

                if (cruiseCount > 1) {
                    s_cruiseRemaining = cruiseCount - 1;
                    s_cruiseMode = true;
                }
                nextInterval = s_cruiseInterval;
            } else if (nextInterval == 1) {
                // Accel sentinel: [1, v_hi, v_lo, a_hi, a_lo, count_hi, count_lo]
                uint16_t vHi = s_lut[s_mbLutIndex++];
                uint16_t vLo = s_lut[s_mbLutIndex++];
                uint16_t aHi = s_lut[s_mbLutIndex++];
                uint16_t aLo = s_lut[s_mbLutIndex++];
                uint16_t countHi = s_lut[s_mbLutIndex++];
                uint16_t countLo = s_lut[s_mbLutIndex++];

                s_accelVelocity = ((uint32_t)vHi << 16) | vLo;
                s_accelRate = (int32_t)(((uint32_t)aHi << 16) | aLo);
                uint32_t accelCount = ((uint32_t)countHi << 16) | countLo;

                // First step uses initial velocity
                uint16_t vel16 = (uint16_t)(s_accelVelocity >> 16);
                nextInterval = (vel16 > 0) ? (uint16_t)((uint32_t)F_CPU / vel16) : 65535;

                if (accelCount > 1) {
                    s_accelRemaining = accelCount - 1;
                    s_accelMode = true;
                }
            }
        }
    } else {
        // Single-move circular double-buffer read
        nextInterval = s_lut[s_lutIndex];
        s_lutIndex++;

        if (s_lutIndex == LUT_HALF_SIZE && s_firstHalfActive) {
            s_firstHalfActive = false;
            if (!s_lutExhausted) s_needRefill = true;
        } else if (s_lutIndex == LUT_TOTAL_SIZE) {
            s_lutIndex = 0;
            s_firstHalfActive = true;
            if (!s_lutExhausted) s_needRefill = true;
        }
    }

    // 4. Handle prescaler switching
    if (s_usingPrescaler8) {
        if (nextInterval < (65535 / 8)) {
            TCCR3B = (1 << WGM32) | (1 << CS30);
            s_usingPrescaler8 = false;
        } else {
            nextInterval >>= 3;
            if (nextInterval < 1) nextInterval = 1;
        }
    } else {
        if (nextInterval >= 65535) {
            TCCR3B = (1 << WGM32) | (1 << CS31);
            s_usingPrescaler8 = true;
            nextInterval >>= 3;
            if (nextInterval < 1) nextInterval = 1;
        }
    }

    OCR3A = nextInterval;

    // 5. Schedule STEP LOW via CompB
    OCR3B = TCNT3 + PULSE_TICKS;
    TIMSK3 |= (1 << OCIE3B);

    // 6. Check if move is complete (single-move mode only)
    if (!s_multiBlockMode && s_stepsRemaining <= 0) {
        s_moveActive = false;
        TIMSK3 &= ~(1 << OCIE3A);
    }
}

// ===== Timer3 CompB ISR: Drive STEP pins LOW =====
ISR(TIMER3_COMPB_vect) {
    *s_domStepPort &= ~s_domStepMask;

    if (s_minStepPendingLow) {
        *s_minStepPort &= ~s_minStepMask;
        s_minStepPendingLow = false;
    }

    TIMSK3 &= ~(1 << OCIE3B);
}

#endif // UNIT_TEST
