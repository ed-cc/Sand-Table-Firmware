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
// On AVR, the profile is pre-computed into a lookup table (LUT) of timer
// intervals so the ISR performs only a single array read per step.

#ifndef SCURVE_PROFILE_H
#define SCURVE_PROFILE_H

#include <stdint.h>

class SCurveProfile {
public:
    SCurveProfile();

    // Plan a move profile.
    // totalSteps: number of steps for the dominant axis (always positive)
    // maxVel:     maximum velocity (steps/sec)
    // maxAccel:   maximum acceleration (steps/sec^2)
    // jerk:       maximum jerk (steps/sec^3)
    void plan(int32_t totalSteps, float maxVel, float maxAccel, float jerk);

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

    // Total planned steps
    int32_t totalSteps() const { return m_totalSteps; }

    // Total move duration in seconds
    float totalTime() const { return m_totalTime; }

    // Check if step index has completed the profile
    bool isComplete(int32_t stepIndex) const { return stepIndex >= m_totalSteps; }

    // Phase durations (for testing/debugging)
    float phaseDuration(uint8_t phase) const;  // phase 0-6
    float phaseStartTime(uint8_t phase) const; // cumulative start time

private:
    // Convert step index to time using velocity integration
    float timeAtStep(int32_t stepIndex) const;

    // Distance (steps) covered in time t from move start
    float distanceAt(float t) const;

    int32_t m_totalSteps;
    float   m_maxVel;      // achieved max velocity (may be less than requested)
    float   m_maxAccel;    // achieved max acceleration (may be less than requested)
    float   m_jerk;
    float   m_totalTime;

    // Phase durations T[0..6] and cumulative boundary times B[0..7]
    float   m_T[7];        // duration of each phase
    float   m_B[8];        // cumulative time at start of each phase; B[7] = total time

    // Velocity at phase boundaries V[0..7]
    float   m_V[8];        // V[0]=0 (start), V[3]=Vmax (cruise start), V[7]=0 (end)

    // Minimum velocity to prevent division by zero (steps/sec)
    static constexpr float MIN_VEL = 1.0f;
};

#endif // SCURVE_PROFILE_H
