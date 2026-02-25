// timer_backend.h - Hardware timer interface for stepper step generation
//
// Provides timer-ISR-driven step pulse generation using ATmega2560 Timer3/4
// in CTC mode with direct PORT register writes. The master ISR (Timer3)
// drives the dominant axis and runs the Bresenham accumulator for the
// minor axis. A companion ISR (Timer3 CompB) drives STEP pins LOW
// after the minimum pulse width.
//
// Two execution modes:
//   Single-move mode (timerBackendStartMove):
//     Double-buffered LUT for CoordinatedStepper / homing moves.
//     Existing path, unchanged.
//
//   Multi-block mode (timerBackendLoadPlan):
//     Linear LUT pre-computed for all queued blocks.
//     BlockMeta array defines per-block Bresenham params and directions.
//     ISR transitions seamlessly between blocks at LUT boundaries.
//
// Usage:
//   1. Call timerBackendInit() in setup() after CoordinatedStepper::init()
//   2. Call timerBackendStartMove() for single moves (homing),
//      or timerBackendLoadPlan() for multi-block planned sequences
//   3. Call timerBackendRefillLUT() periodically for single-move mode only
//   4. The ISR fires automatically; check timerBackendIsComplete()

#ifndef TIMER_BACKEND_H
#define TIMER_BACKEND_H

#include "coordinated_stepper.h"
#include "values.h"

#ifndef UNIT_TEST

// ===== BlockMeta: per-block ISR execution metadata =====
struct BlockMeta {
    uint16_t lut_start;      // index into lut[] where this block begins
    uint16_t lut_count;      // number of dominant steps for this block
    int8_t   dir_theta;      // +1 or -1
    int8_t   dir_radius;     // +1 or -1
    int32_t  bres_dom;       // dominant step count (Bresenham)
    int32_t  bres_minor;     // minor step count (Bresenham)
    bool     theta_dominant; // true if theta is the dominant axis for this block
};

// Initialise Timer3 and Timer4 in CTC mode. Call once in setup().
void timerBackendInit();

// ===== Single-move mode (backward compatible, for homing) =====

// Start ISR-driven stepping for the current move planned in the stepper.
// Precomputes the first LUT block and enables the timer interrupt.
void timerBackendStartMove(CoordinatedStepper& stepper);

// Refill the LUT double-buffer from the main loop.
// Returns true if the move is still active and more refills may be needed.
bool timerBackendRefillLUT(CoordinatedStepper& stepper);

// ===== Multi-block mode (lookahead planner) =====

// Return a pointer to the internal LUT buffer so callers can write intervals
// directly, avoiding a redundant copy. The returned buffer has room for
// AVR_MAX_PRECOMPUTE_STEPS entries (uint16_t each). Only safe to call when
// the ISR is not actively reading the multi-block LUT (i.e. before
// timerBackendLoadPlan, or after timerBackendIsComplete).
uint16_t* timerBackendGetLutBuffer();

// Load a pre-computed plan into the ISR for seamless multi-block execution.
// The caller must have already written interval data into the buffer returned
// by timerBackendGetLutBuffer() before calling this function.
// metas:      array of BlockMeta (one per block, in execution order)
// blockCount: number of blocks
// lutSize:    total entries written into the LUT buffer
// thetaAxis:  pointer to theta StepperAxis
// radiusAxis: pointer to radius StepperAxis
void timerBackendLoadPlan(const BlockMeta* metas, uint8_t blockCount,
                          uint16_t lutSize,
                          StepperAxis* thetaAxis, StepperAxis* radiusAxis);

// ===== Common API =====

// Stop the timer ISRs (disables step interrupts). Call on emergency stop.
void timerBackendStop();

// Check if the ISR has finished all steps
bool timerBackendIsComplete();

// Request abort: ISR will enter forced deceleration on next tick.
void timerBackendAbort();

// Check if the ISR stalled (forced decel completed or LUT exhausted)
bool timerBackendIsStalled();

// Diagnostic: total ISR fires since last move start
int32_t timerBackendISRCount();

// Diagnostic: steps remaining in current move
int32_t timerBackendStepsRemaining();

// LUT configuration
// Single-move mode uses first LUT_TOTAL_SIZE entries with double-buffer.
// Multi-block mode uses full AVR_MAX_PRECOMPUTE_STEPS entries linearly.
static const int16_t LUT_HALF_SIZE = 512;
static const int16_t LUT_TOTAL_SIZE = LUT_HALF_SIZE * 2;

// Minimum pulse width in timer ticks (2 µs at 16 MHz = 32 ticks)
static const uint16_t PULSE_TICKS = 32;

// Prescaler threshold: below this speed (steps/sec), use prescaler=8
static const uint16_t PRESCALER_THRESHOLD_SPEED = 244;

#endif // UNIT_TEST

#endif // TIMER_BACKEND_H
