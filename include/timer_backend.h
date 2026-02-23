// timer_backend.h - Hardware timer interface for stepper step generation
//
// Provides timer-ISR-driven step pulse generation using ATmega2560 Timer3/4
// in CTC mode with direct PORT register writes. The master ISR (Timer3)
// drives the dominant axis and runs the Bresenham accumulator for the
// minor axis. A companion ISR (Timer3 CompB) drives STEP pins LOW
// after the minimum pulse width.
//
// Usage:
//   1. Call timerBackendInit() in setup() after CoordinatedStepper::init()
//   2. Call timerBackendStartMove() when a new move begins
//   3. Call timerBackendRefillLUT() periodically in the main loop
//   4. The ISR fires automatically; check CoordinatedStepper::isComplete()

#ifndef TIMER_BACKEND_H
#define TIMER_BACKEND_H

#include "coordinated_stepper.h"

#ifndef UNIT_TEST

// Initialise Timer3 and Timer4 in CTC mode. Call once in setup().
void timerBackendInit();

// Start ISR-driven stepping for the current move planned in the stepper.
// Precomputes the first LUT block and enables the timer interrupt.
void timerBackendStartMove(CoordinatedStepper& stepper);

// Stop the timer ISRs (disables step interrupts). Call on emergency stop.
void timerBackendStop();

// Refill the LUT double-buffer from the main loop.
// Returns true if the move is still active and more refills may be needed.
bool timerBackendRefillLUT(CoordinatedStepper& stepper);

// Check if the ISR has finished all steps
bool timerBackendIsComplete();

// Diagnostic: total ISR fires since last move start
int32_t timerBackendISRCount();

// Diagnostic: steps remaining in current move
int32_t timerBackendStepsRemaining();

// LUT configuration
static const int16_t LUT_HALF_SIZE = 512;
static const int16_t LUT_TOTAL_SIZE = LUT_HALF_SIZE * 2;

// Minimum pulse width in timer ticks (2 µs at 16 MHz = 32 ticks)
static const uint16_t PULSE_TICKS = 32;

// Prescaler threshold: below this speed (steps/sec), use prescaler=8
static const uint16_t PRESCALER_THRESHOLD_SPEED = 244;

#endif // UNIT_TEST

#endif // TIMER_BACKEND_H
