// trajectory_planner.h - Top-level queue API for multi-block trajectory planning
//
// Bridges the planner buffer (physical-space junction planning) with the
// timer backend ISR (step execution). Provides the public interface for
// queueing moves, synchronising, feed hold, resume, and abort.
//
// Usage:
//   1. Call init() once in setup()
//   2. Call queue_move() for each move segment
//   3. Call end_of_program() after last move (or synchronise() to wait)
//   4. Call run() from main loop for stall detection and state management

#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "planner_buffer.h"
#include "scurve_profile.h"
#include "stepper_axis.h"
#include "values.h"

#ifndef UNIT_TEST
#include "timer_backend.h"
#endif

class TrajectoryPlanner {
public:
    TrajectoryPlanner();

    // Initialise with axis pointers. Call once in setup().
    void init(StepperAxis* thetaAxis, StepperAxis* radiusAxis);

    // Queue a polar move for blended execution.
    // delta_r_mm:     radius change [mm]
    // delta_theta_deg: theta change [degrees]
    // feed_mmps:      feed rate [mm/s] (physical distance rate)
    // R_current_mm:   current radius position [mm]
    void queue_move(float delta_r_mm, float delta_theta_deg,
                    float feed_mmps, float R_current_mm);

    // Mark end of program: the last queued block gets exit_speed=0,
    // then start execution of all buffered blocks.
    void end_of_program();

    // Block until all queued moves are complete.
    // Calls end_of_program() if not already called, then polls until done.
    void synchronise();

    // Alias for synchronise()
    void flush() { synchronise(); }

    // True if the ISR is currently executing moves
    bool is_moving() const;

    // True if the planner buffer cannot accept more blocks
    bool buffer_full() const;

    // True if there are queued blocks not yet executing
    bool has_queued_blocks() const;

    // True if not executing and buffer is empty (all motion done)
    bool is_idle() const;

    // Feed hold: decelerate to stop at the end of the current block.
    void feed_hold();

    // Resume after feed hold: re-start execution from where we left off.
    void resume();

    // Emergency abort: forced deceleration, discard all queued blocks.
    void abort();

    // Call from main loop. Handles stall detection and diagnostics.
    void run();

private:
    // Prepare BlockMeta array + LUT from current planner buffer contents,
    // then call timerBackendLoadPlan() to start ISR execution.
    void _start_execution();

    PlannerBuffer  m_buffer;
    SCurveProfile  m_profile;  // reused for each block during preparation
    StepperAxis*   m_thetaAxis;
    StepperAxis*   m_radiusAxis;

    bool m_executing;      // true while ISR is running a plan
    bool m_feedHoldActive; // true if feed hold was requested

    // Batch-split state for oversized blocks whose LUT entries exceed
    // AVR_MAX_PRECOMPUTE_STEPS. The full profile is planned once, then
    // fed to the ISR in batches of raw per-step intervals.
    void _start_split_batch();

    bool          m_splitActive;
    SCurveProfile m_splitProfile;    // full profile for the oversized block
    int32_t       m_splitTotalDom;   // total dominant steps in full profile
    int32_t       m_splitDomFired;   // cumulative dom steps sent to ISR
    int32_t       m_splitTotalMinor; // total minor steps for Bresenham
    int32_t       m_splitMinorFired; // cumulative minor steps sent
    int8_t        m_splitDirTheta;
    int8_t        m_splitDirRadius;
    bool          m_splitThetaDom;
};

#endif // TRAJECTORY_PLANNER_H
