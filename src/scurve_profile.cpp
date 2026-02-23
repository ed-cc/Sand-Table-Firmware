// scurve_profile.cpp - 7-phase S-curve velocity profile for stepper motors
//
// Supports asymmetric profiles with arbitrary boundary velocities (v_start, v_end)
// for move blending. When v_start=0 and v_end=0 (default), produces the original
// symmetric stop-to-stop profile.

#include "scurve_profile.h"
#include <math.h>

SCurveProfile::SCurveProfile()
    : m_totalSteps(0)
    , m_maxVel(0)
    , m_jerk(0)
    , m_totalTime(0)
    , m_accelRate(0)
    , m_decelRate(0)
{
    for (int i = 0; i < 7; i++) m_T[i] = 0;
    for (int i = 0; i < 8; i++) { m_B[i] = 0; m_V[i] = 0; }
}

// Compute distance covered during a single 3-phase acceleration ramp
// (jerk-up + const-accel + jerk-down) starting from velocity = 0.
// For a ramp starting at v_base, add v_base * (2*T1 + T2) to this result.
static float accelPhaseDistance(float T1, float T2, float T3, float J) {
    float d1 = J * T1 * T1 * T1 / 6.0f;
    float v1 = J * T1 * T1 / 2.0f;
    float a1 = J * T1;
    float d2 = v1 * T2 + a1 * T2 * T2 / 2.0f;
    float v2 = v1 + a1 * T2;
    float d3 = v2 * T3 + a1 * T3 * T3 / 2.0f - J * T3 * T3 * T3 / 6.0f;
    return d1 + d2 + d3;
}

// Compute T1 and T2 for a ramp that covers delta_v velocity change.
static void computeRampParams(float delta_v, float maxAccel, float jerk,
                               float& T1_out, float& T2_out) {
    if (delta_v <= 0.0f) {
        T1_out = 0.0f;
        T2_out = 0.0f;
        return;
    }

    float vJerkOnly = maxAccel * maxAccel / jerk;

    if (vJerkOnly >= delta_v) {
        T1_out = sqrtf(delta_v / jerk);
        T2_out = 0.0f;
    } else {
        T1_out = maxAccel / jerk;
        T2_out = (delta_v - vJerkOnly) / maxAccel;
    }
}

// Distance for a ramp starting at v_base accelerating by delta_v.
static float rampDistance(float v_base, float T1, float T2, float jerk) {
    float T_total = 2.0f * T1 + T2;
    return v_base * T_total + accelPhaseDistance(T1, T2, T1, jerk);
}

void SCurveProfile::plan(int32_t totalSteps, float maxVel, float maxAccel, float jerk,
                          float v_start, float v_end) {
    m_totalSteps = totalSteps;
    m_jerk = jerk;

    if (totalSteps <= 0 || maxVel <= 0 || maxAccel <= 0 || jerk <= 0) {
        m_maxVel = 0;
        m_accelRate = 0;
        m_decelRate = 0;
        m_totalTime = 0;
        for (int i = 0; i < 7; i++) m_T[i] = 0;
        for (int i = 0; i < 8; i++) { m_B[i] = 0; m_V[i] = 0; }
        return;
    }

    // Clamp boundary velocities
    if (v_start < 0.0f) v_start = 0.0f;
    if (v_end < 0.0f) v_end = 0.0f;
    if (v_start > maxVel) v_start = maxVel;
    if (v_end > maxVel) v_end = maxVel;

    float achievedVmax = maxVel;

    // Compute accel ramp params (v_start -> achievedVmax)
    float T1a, T2a;
    computeRampParams(achievedVmax - v_start, maxAccel, jerk, T1a, T2a);
    float dAccel = rampDistance(v_start, T1a, T2a, jerk);

    // Compute decel ramp params (achievedVmax -> v_end)
    float T1d, T2d;
    computeRampParams(achievedVmax - v_end, maxAccel, jerk, T1d, T2d);
    float dDecel = rampDistance(v_end, T1d, T2d, jerk);

    float dTotal = dAccel + dDecel;

    if (dTotal > (float)totalSteps) {
        // Not enough distance for full speed. Binary search for achievable Vmax.
        float vLo = v_start > v_end ? v_start : v_end;
        if (vLo < MIN_VEL) vLo = MIN_VEL;
        float vHi = achievedVmax;

        for (int iter = 0; iter < 30; iter++) {
            float vMid = (vLo + vHi) / 2.0f;

            float t1a_t, t2a_t, t1d_t, t2d_t;
            computeRampParams(vMid - v_start, maxAccel, jerk, t1a_t, t2a_t);
            computeRampParams(vMid - v_end, maxAccel, jerk, t1d_t, t2d_t);

            float d = rampDistance(v_start, t1a_t, t2a_t, jerk)
                    + rampDistance(v_end, t1d_t, t2d_t, jerk);

            if (d > (float)totalSteps) {
                vHi = vMid;
            } else {
                vLo = vMid;
            }
        }

        achievedVmax = vLo;

        // Recompute ramp params with final Vmax
        computeRampParams(achievedVmax - v_start, maxAccel, jerk, T1a, T2a);
        computeRampParams(achievedVmax - v_end, maxAccel, jerk, T1d, T2d);
        dAccel = rampDistance(v_start, T1a, T2a, jerk);
        dDecel = rampDistance(v_end, T1d, T2d, jerk);
    }

    m_maxVel = achievedVmax;
    m_accelRate = jerk * T1a;
    m_decelRate = jerk * T1d;

    // Cruise phase duration
    float dCruise = (float)totalSteps - dAccel - dDecel;
    float T4 = (dCruise > 0 && achievedVmax > MIN_VEL) ? dCruise / achievedVmax : 0;

    // Assign phase durations (asymmetric)
    m_T[0] = T1a;  // Phase 1: jerk up (accel)
    m_T[1] = T2a;  // Phase 2: const accel
    m_T[2] = T1a;  // Phase 3: jerk down (accel)
    m_T[3] = T4;   // Phase 4: cruise
    m_T[4] = T1d;  // Phase 5: jerk down (decel start)
    m_T[5] = T2d;  // Phase 6: const decel
    m_T[6] = T1d;  // Phase 7: jerk up (decel end)

    // Compute boundary times
    m_B[0] = 0;
    for (int i = 0; i < 7; i++) {
        m_B[i + 1] = m_B[i] + m_T[i];
    }
    m_totalTime = m_B[7];

    // Compute boundary velocities
    m_V[0] = v_start;
    m_V[1] = v_start + jerk * T1a * T1a / 2.0f;
    m_V[2] = m_V[1] + m_accelRate * T2a;
    m_V[3] = achievedVmax;
    m_V[4] = achievedVmax;
    m_V[5] = achievedVmax - jerk * T1d * T1d / 2.0f;
    m_V[6] = m_V[5] - m_decelRate * T2d;
    m_V[7] = v_end;
}

void SCurveProfile::planPhysical(float phys_distance_mm,
                                  float v_start_mmps, float v_nominal_mmps,
                                  float v_end_mmps,
                                  float a_max_mmps2, float j_max_mmps3,
                                  float steps_per_mm) {
    int32_t totalSteps = (int32_t)(phys_distance_mm * steps_per_mm + 0.5f);
    float maxVel_sps   = v_nominal_mmps * steps_per_mm;
    float maxAccel_sps = a_max_mmps2 * steps_per_mm;
    float jerk_sps     = j_max_mmps3 * steps_per_mm;
    float vs_sps       = v_start_mmps * steps_per_mm;
    float ve_sps       = v_end_mmps * steps_per_mm;

    plan(totalSteps, maxVel_sps, maxAccel_sps, jerk_sps, vs_sps, ve_sps);
}

uint32_t SCurveProfile::steps_to_stop(float v_current, float a_max, float steps_per_mm) {
    if (v_current <= 0.0f || a_max <= 0.0f) return 0;
    float d_mm = (v_current * v_current) / (2.0f * a_max);
    uint32_t steps = (uint32_t)(d_mm * steps_per_mm + 1.0f);
    return steps;
}

float SCurveProfile::velocityAt(float t) const {
    if (m_totalSteps <= 0 || t < 0) return 0;
    if (t >= m_totalTime) return m_V[7];

    // Find which phase we're in
    uint8_t ph = 6;
    for (uint8_t i = 0; i < 7; i++) {
        if (t < m_B[i + 1]) {
            ph = i;
            break;
        }
    }

    float dt = t - m_B[ph];
    float v;

    switch (ph) {
        case 0:  // Jerk up: v = V[0] + J*dt^2/2
            v = m_V[0] + m_jerk * dt * dt / 2.0f;
            break;
        case 1:  // Const accel: v = V[1] + accelRate*dt
            v = m_V[1] + m_accelRate * dt;
            break;
        case 2:  // Jerk down: v = V[2] + accelRate*dt - J*dt^2/2
            v = m_V[2] + m_accelRate * dt - m_jerk * dt * dt / 2.0f;
            break;
        case 3:  // Cruise: v = Vmax
            v = m_V[3];
            break;
        case 4:  // Jerk down (decel start): v = Vmax - J*dt^2/2
            v = m_V[4] - m_jerk * dt * dt / 2.0f;
            break;
        case 5:  // Const decel: v = V[5] - decelRate*dt
            v = m_V[5] - m_decelRate * dt;
            break;
        case 6:  // Jerk up (decel end): v = V[6] - decelRate*dt + J*dt^2/2
            v = m_V[6] - m_decelRate * dt + m_jerk * dt * dt / 2.0f;
            break;
        default:
            v = 0;
            break;
    }

    return (v > MIN_VEL) ? v : MIN_VEL;
}

float SCurveProfile::distanceAt(float t) const {
    if (m_totalSteps <= 0 || t <= 0) return 0;
    if (t >= m_totalTime) return (float)m_totalSteps;

    float d = 0;
    float tRemain = t;

    for (uint8_t ph = 0; ph < 7 && tRemain > 0; ph++) {
        float dt = (tRemain < m_T[ph]) ? tRemain : m_T[ph];
        tRemain -= dt;

        switch (ph) {
            case 0:  // Jerk up: d = V[0]*dt + J*dt^3/6
                d += m_V[0] * dt + m_jerk * dt * dt * dt / 6.0f;
                break;
            case 1:  // Const accel: d = V[1]*dt + accelRate*dt^2/2
                d += m_V[1] * dt + m_accelRate * dt * dt / 2.0f;
                break;
            case 2:  // Jerk down: d = V[2]*dt + accelRate*dt^2/2 - J*dt^3/6
                d += m_V[2] * dt + m_accelRate * dt * dt / 2.0f - m_jerk * dt * dt * dt / 6.0f;
                break;
            case 3:  // Cruise: d = Vmax*dt
                d += m_V[3] * dt;
                break;
            case 4:  // Jerk down (decel): d = Vmax*dt - J*dt^3/6
                d += m_V[4] * dt - m_jerk * dt * dt * dt / 6.0f;
                break;
            case 5:  // Const decel: d = V[5]*dt - decelRate*dt^2/2
                d += m_V[5] * dt - m_decelRate * dt * dt / 2.0f;
                break;
            case 6:  // Jerk up (decel end): d = V[6]*dt - decelRate*dt^2/2 + J*dt^3/6
                d += m_V[6] * dt - m_decelRate * dt * dt / 2.0f + m_jerk * dt * dt * dt / 6.0f;
                break;
        }
    }

    return d;
}

float SCurveProfile::timeAtStep(int32_t stepIndex) const {
    if (m_totalSteps <= 0) return 0;
    if (stepIndex >= m_totalSteps) return m_totalTime;
    if (stepIndex <= 0) return 0;

    float target = (float)stepIndex;

    // Binary search for time where distanceAt(t) = stepIndex
    float lo = 0;
    float hi = m_totalTime;

    for (int iter = 0; iter < 40; iter++) {
        float mid = (lo + hi) / 2.0f;
        float d = distanceAt(mid);
        if (d < target) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    return (lo + hi) / 2.0f;
}

uint16_t SCurveProfile::intervalAt(int32_t stepIndex, uint32_t tickFreq) const {
    if (m_totalSteps <= 0) return 65535;
    if (stepIndex >= m_totalSteps) return 65535;

    float t = timeAtStep(stepIndex);
    float v = velocityAt(t);
    if (v < MIN_VEL) v = MIN_VEL;

    float interval = (float)tickFreq / v;
    if (interval > 65535.0f) return 65535;
    if (interval < 1.0f) return 1;
    return (uint16_t)(interval + 0.5f);
}

int32_t SCurveProfile::precomputeLUT(uint16_t* lut, int32_t maxEntries, uint32_t tickFreq) const {
    return computeIntervalBlock(0, lut, maxEntries, tickFreq);
}

int32_t SCurveProfile::computeIntervalBlock(int32_t startStep, uint16_t* lut, int32_t count, uint32_t tickFreq) const {
    if (count <= 0 || m_totalSteps <= 0 || startStep >= m_totalSteps) return 0;

    int32_t toCompute = count;
    if (startStep + toCompute > m_totalSteps) toCompute = m_totalSteps - startStep;

    float t = (startStep == 0) ? 0.0f : timeAtStep(startStep);
    float fTickFreq = (float)tickFreq;

    for (int32_t i = 0; i < toCompute; i++) {
        float v = velocityAt(t);
        if (v < MIN_VEL) v = MIN_VEL;

        float interval = fTickFreq / v;
        if (interval > 65535.0f) lut[i] = 65535;
        else if (interval < 1.0f) lut[i] = 1;
        else lut[i] = (uint16_t)(interval + 0.5f);

        t += 1.0f / v;
    }
    return toCompute;
}

float SCurveProfile::phaseDuration(uint8_t phase) const {
    if (phase >= 7) return 0;
    return m_T[phase];
}

float SCurveProfile::phaseStartTime(uint8_t phase) const {
    if (phase >= 8) return m_totalTime;
    return m_B[phase];
}
