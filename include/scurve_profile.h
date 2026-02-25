// scurve_profile.h - 7-phase S-curve velocity profile for stepper motors
//
// Computes jerk-limited acceleration/deceleration profiles that produce
// smooth motion without exciting mechanical resonances. The profile has
// up to 7 phases:
//
//   Phase 1: jerk +J  (accel ramps up)
//   Phase 2: jerk  0  (constant accel)
//   Phase 3: jerk -J  (accel ramps down to zero)
//   Phase 4: jerk  0  (cruise at Vmax)
//   Phase 5: jerk -J  (decel ramps up)
//   Phase 6: jerk  0  (constant decel)
//   Phase 7: jerk +J  (decel ramps down to zero)
//
// Short moves degrade gracefully: T4=0 (no cruise), T2=T6=0 (triangular),
// or reduced Vmax/Amax for very short moves.
//
// Supports arbitrary boundary velocities (v_start, v_end) for move blending.
// Default v_start=0, v_end=0 preserves original stop-to-stop behavior.
//
// On AVR, the profile is pre-computed into a lookup table (LUT) of timer
// intervals so the ISR performs only a single array read per step.

#ifndef SCURVE_PROFILE_H
#define SCURVE_PROFILE_H

#include <stdint.h>

class SCurveProfile {
public:
    SCurveProfile();

    // Plan a move profile with optional boundary velocities.
    // totalSteps: number of steps for the dominant axis (always positive)
    // maxVel:     maximum velocity (steps/sec)
    // maxAccel:   maximum acceleration (steps/sec^2)
    // jerk:       maximum jerk (steps/sec^3)
    // v_start:    velocity at move start (steps/sec, default 0)
    // v_end:      velocity at move end (steps/sec, default 0)
    void plan(int32_t totalSteps, float maxVel, float maxAccel, float jerk,
              float v_start = 0.0f, float v_end = 0.0f);

    // Plan in physical units, then convert to step-space internally.
    // phys_distance_mm: physical distance of the move [mm]
    // v_start_mmps:     entry velocity [mm/s]
    // v_nominal_mmps:   target cruise velocity [mm/s]
    // v_end_mmps:       exit velocity [mm/s]
    // a_max_mmps2:      max acceleration [mm/s^2]
    // j_max_mmps3:      max jerk [mm/s^3]
    // steps_per_mm:     step rate for dominant axis [steps/mm]
    void planPhysical(float phys_distance_mm,
                      float v_start_mmps, float v_nominal_mmps, float v_end_mmps,
                      float a_max_mmps2, float j_max_mmps3,
                      float steps_per_mm);

    // Emergency: steps needed to decelerate from v to 0 (trapezoidal, not S-curve).
    static uint32_t steps_to_stop(float v_current, float a_max, float steps_per_mm);

    // Get velocity at a given time (seconds from move start).
    // Returns velocity in steps/sec.
    float velocityAt(float t) const;

    // Get the timer interval for a given step index.
    // tickFreq: timer frequency in Hz (e.g. 16000000 for 16 MHz AVR)
    // Returns timer ticks per step. Clamped to uint16_t range.
    uint16_t intervalAt(int32_t stepIndex, uint32_t tickFreq) const;

    // Pre-compute a lookup table of timer intervals for AVR ISR consumption.
    // lut:        output array (caller-allocated)
    // maxEntries: size of lut array
    // tickFreq:   timer frequency in Hz
    // Returns number of entries written (min of totalSteps and maxEntries).
    int32_t precomputeLUT(uint16_t* lut, int32_t maxEntries, uint32_t tickFreq) const;

    // Compute a block of timer intervals starting at a given step index.
    // Uses incremental time tracking (one binary search at the start, then
    // O(1) per step) — much faster than calling intervalAt() individually.
    // lut:        output array (caller-allocated)
    // count:      max entries to compute
    // tickFreq:   timer frequency in Hz
    // Returns number of entries written.
    int32_t computeIntervalBlock(int32_t startStep, uint16_t* lut, int32_t count, uint32_t tickFreq) const;

    // Pre-compute a compressed LUT for multi-block trajectory planning.
    // Compresses the cruise phase (phase 4) into a 4-entry sentinel [0, interval, count_hi, count_lo]
    // instead of storing thousands of identical intervals. Other phases stored per-step as normal.
    // lut:        output array (caller-allocated)
    // maxEntries: size of lut array
    // tickFreq:   timer frequency in Hz
    // Returns number of compressed LUT entries written.
    int32_t precomputeCompressedLUT(uint16_t* lut, int32_t maxEntries, uint32_t tickFreq) const;

    // Return the number of LUT entries needed in compressed format.
    // Compresses cruise (phase 3) and constant-accel/decel (phases 1, 5)
    // into sentinels, so far fewer entries than totalSteps().
    // Used for overflow checking before pre-computation.
    int32_t lutEntryCount() const;

    // Phase boundary info for compressed LUT planning.
    struct PhaseBoundaries {
        int32_t jerk0Steps;       // steps in phase 0 (jerk up)
        int32_t constAccelSteps;  // steps in phase 1 (constant accel)
        int32_t jerk2Steps;       // steps in phase 2 (jerk down)
        int32_t cruiseSteps;      // steps in phase 3 (cruise)
        int32_t jerk4Steps;       // steps in phase 4 (jerk down, decel start)
        int32_t constDecelSteps;  // steps in phase 5 (constant decel)
        int32_t jerk6Steps;       // steps in phase 6 (jerk up, decel end)
    };

    // Compute step counts for each of the 7 phases.
    void phaseBoundaries(PhaseBoundaries& out) const;

    // Total planned steps
    int32_t totalSteps() const { return m_totalSteps; }

    // Total move duration in seconds
    float totalTime() const { return m_totalTime; }

    // Check if step index has completed the profile
    bool isComplete(int32_t stepIndex) const { return stepIndex >= m_totalSteps; }

    // Phase durations (for testing/debugging)
    float phaseDuration(uint8_t phase) const;  // phase 0-6
    float phaseStartTime(uint8_t phase) const; // cumulative start time

    // Boundary velocities (for testing)
    float startVelocity() const { return m_V[0]; }
    float endVelocity() const { return m_V[7]; }
    float peakVelocity() const { return m_maxVel; }

private:
    // Convert step index to time using velocity integration
    float timeAtStep(int32_t stepIndex) const;

    // Distance (steps) covered in time t from move start
    float distanceAt(float t) const;

    int32_t m_totalSteps;
    float   m_maxVel;      // achieved max velocity (may be less than requested)
    float   m_jerk;
    float   m_totalTime;

    // Achieved acceleration for accel and decel ramps (may differ in asymmetric profiles)
    float   m_accelRate;   // J * T1_accel: achieved acceleration during phases 0-2
    float   m_decelRate;   // J * T1_decel: achieved acceleration during phases 4-6

    // Phase durations T[0..6] and cumulative boundary times B[0..7]
    float   m_T[7];        // duration of each phase
    float   m_B[8];        // cumulative time at start of each phase; B[7] = total time

    // Velocity at phase boundaries V[0..7]
    float   m_V[8];        // V[0]=v_start, V[3]=Vmax (cruise start), V[7]=v_end

    // Minimum velocity to prevent division by zero (steps/sec)
    static constexpr float MIN_VEL = 1.0f;
};

#endif // SCURVE_PROFILE_H
