// scurve_profile.cpp - 7-phase S-curve velocity profile for stepper motors

#include "scurve_profile.h"
#include <math.h>

SCurveProfile::SCurveProfile()
    : m_totalSteps(0)
    , m_maxVel(0)
    , m_maxAccel(0)
    , m_jerk(0)
    , m_totalTime(0)
{
    for (int i = 0; i < 7; i++) m_T[i] = 0;
    for (int i = 0; i < 8; i++) { m_B[i] = 0; m_V[i] = 0; }
}

// Compute distance covered during a single S-curve acceleration phase trio
// (jerk-up + const-accel + jerk-down) given phase durations and jerk.
static float accelPhaseDistance(float T1, float T2, float T3, float J) {
    // Phase 1 (jerk up): d1 = J*T1^3/6
    float d1 = J * T1 * T1 * T1 / 6.0f;
    // Velocity at end of phase 1:
    float v1 = J * T1 * T1 / 2.0f;
    // Acceleration at end of phase 1:
    float a1 = J * T1;
    // Phase 2 (const accel): d2 = v1*T2 + a1*T2^2/2
    float d2 = v1 * T2 + a1 * T2 * T2 / 2.0f;
    // Velocity at end of phase 2:
    float v2 = v1 + a1 * T2;
    // Phase 3 (jerk down): d3 = v2*T3 + a1*T3^2/2 - J*T3^3/6
    float d3 = v2 * T3 + a1 * T3 * T3 / 2.0f - J * T3 * T3 * T3 / 6.0f;
    return d1 + d2 + d3;
}

void SCurveProfile::plan(int32_t totalSteps, float maxVel, float maxAccel, float jerk) {
    m_totalSteps = totalSteps;
    m_jerk = jerk;

    if (totalSteps <= 0 || maxVel <= 0 || maxAccel <= 0 || jerk <= 0) {
        m_maxVel = 0;
        m_maxAccel = 0;
        m_totalTime = 0;
        for (int i = 0; i < 7; i++) m_T[i] = 0;
        for (int i = 0; i < 8; i++) { m_B[i] = 0; m_V[i] = 0; }
        return;
    }

    // Step 1: Compute jerk phase duration T1 = Amax / J
    float T1 = maxAccel / jerk;

    // Step 2: Compute velocity gained during jerk phases only (T2=0)
    // During phases 1+3 (T1 each), velocity gain = J * T1^2 = Amax^2 / J
    float vJerkOnly = jerk * T1 * T1;  // = Amax^2 / J

    float achievedVmax;
    float T2;

    if (vJerkOnly >= maxVel) {
        // Can't even reach Amax before hitting Vmax.
        // Reduce T1 so that v = J*T1^2 = Vmax
        T1 = sqrtf(maxVel / jerk);
        T2 = 0;
        achievedVmax = maxVel;
    } else {
        // T2 = time at constant Amax to reach Vmax
        // Vmax = vJerkOnly + Amax * T2
        T2 = (maxVel - vJerkOnly) / maxAccel;
        achievedVmax = maxVel;
    }

    // Step 3: Compute distance needed for accel and decel phases
    // Accel = phases 1,2,3 (T1, T2, T1). Decel = phases 5,6,7 (same durations).
    float dAccel = accelPhaseDistance(T1, T2, T1, jerk);
    float dTotal = dAccel * 2.0f;  // accel + decel (symmetric)

    if (dTotal > (float)totalSteps) {
        // Not enough distance for full speed. Reduce Vmax iteratively.
        // Binary search for the achievable Vmax.
        float vLo = MIN_VEL;
        float vHi = achievedVmax;

        for (int iter = 0; iter < 30; iter++) {
            float vMid = (vLo + vHi) / 2.0f;

            // Recompute T1, T2 for this vMid
            float t1, t2;
            float vJerk = jerk * (maxAccel / jerk) * (maxAccel / jerk);  // Amax^2/J
            if (vJerk >= vMid) {
                t1 = sqrtf(vMid / jerk);
                t2 = 0;
            } else {
                t1 = maxAccel / jerk;
                t2 = (vMid - vJerk) / maxAccel;
            }

            float d = accelPhaseDistance(t1, t2, t1, jerk) * 2.0f;
            if (d > (float)totalSteps) {
                vHi = vMid;
            } else {
                vLo = vMid;
            }
        }

        achievedVmax = vLo;
        // Recompute T1, T2 with final Vmax
        float vJerk = jerk * (maxAccel / jerk) * (maxAccel / jerk);
        if (vJerk >= achievedVmax) {
            T1 = sqrtf(achievedVmax / jerk);
            T2 = 0;
        } else {
            T1 = maxAccel / jerk;
            T2 = (achievedVmax - vJerk) / maxAccel;
        }
        dAccel = accelPhaseDistance(T1, T2, T1, jerk);
    }

    m_maxVel = achievedVmax;
    m_maxAccel = jerk * T1;  // achieved acceleration

    // Step 4: Compute cruise phase duration
    float dCruise = (float)totalSteps - 2.0f * dAccel;
    float T4 = (dCruise > 0 && achievedVmax > MIN_VEL) ? dCruise / achievedVmax : 0;

    // Step 5: Assign phase durations (symmetric profile)
    m_T[0] = T1;  // Phase 1: jerk up
    m_T[1] = T2;  // Phase 2: const accel
    m_T[2] = T1;  // Phase 3: jerk down
    m_T[3] = T4;  // Phase 4: cruise
    m_T[4] = T1;  // Phase 5: jerk down (decel start)
    m_T[5] = T2;  // Phase 6: const decel
    m_T[6] = T1;  // Phase 7: jerk up (decel end)

    // Step 6: Compute boundary times
    m_B[0] = 0;
    for (int i = 0; i < 7; i++) {
        m_B[i + 1] = m_B[i] + m_T[i];
    }
    m_totalTime = m_B[7];

    // Step 7: Compute boundary velocities
    m_V[0] = 0;                                          // start
    m_V[1] = jerk * T1 * T1 / 2.0f;                     // end phase 1
    m_V[2] = m_V[1] + m_maxAccel * T2;                   // end phase 2
    m_V[3] = achievedVmax;                                // end phase 3 / start cruise
    m_V[4] = achievedVmax;                                // end cruise
    m_V[5] = achievedVmax - jerk * T1 * T1 / 2.0f;       // end phase 5
    m_V[6] = m_V[5] - m_maxAccel * T2;                   // end phase 6
    m_V[7] = 0;                                          // end (should be ~0)
}

float SCurveProfile::velocityAt(float t) const {
    if (m_totalSteps <= 0 || t < 0) return 0;
    if (t >= m_totalTime) return 0;

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
        case 0:  // Jerk up: v = J*dt^2/2
            v = m_jerk * dt * dt / 2.0f;
            break;
        case 1:  // Const accel: v = V1 + Amax*dt
            v = m_V[1] + m_maxAccel * dt;
            break;
        case 2:  // Jerk down: v = V2 + Amax*dt - J*dt^2/2
            v = m_V[2] + m_maxAccel * dt - m_jerk * dt * dt / 2.0f;
            break;
        case 3:  // Cruise: v = Vmax
            v = m_V[3];
            break;
        case 4:  // Jerk down (decel start): v = Vmax - J*dt^2/2
            v = m_V[4] - m_jerk * dt * dt / 2.0f;
            break;
        case 5:  // Const decel: v = V5 - Amax*dt
            v = m_V[5] - m_maxAccel * dt;
            break;
        case 6:  // Jerk up (decel end): v = V6 - Amax*dt + J*dt^2/2
            v = m_V[6] - m_maxAccel * dt + m_jerk * dt * dt / 2.0f;
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
            case 0:  // Jerk up: d = J*dt^3/6
                d += m_jerk * dt * dt * dt / 6.0f;
                break;
            case 1:  // Const accel: d = V1*dt + Amax*dt^2/2
                d += m_V[1] * dt + m_maxAccel * dt * dt / 2.0f;
                break;
            case 2:  // Jerk down: d = V2*dt + Amax*dt^2/2 - J*dt^3/6
                d += m_V[2] * dt + m_maxAccel * dt * dt / 2.0f - m_jerk * dt * dt * dt / 6.0f;
                break;
            case 3:  // Cruise: d = Vmax*dt
                d += m_V[3] * dt;
                break;
            case 4:  // Jerk down (decel): d = Vmax*dt - J*dt^3/6
                d += m_V[4] * dt - m_jerk * dt * dt * dt / 6.0f;
                break;
            case 5:  // Const decel: d = V5*dt - Amax*dt^2/2
                d += m_V[5] * dt - m_maxAccel * dt * dt / 2.0f;
                break;
            case 6:  // Jerk up (decel end): d = V6*dt - Amax*dt^2/2 + J*dt^3/6
                d += m_V[6] * dt - m_maxAccel * dt * dt / 2.0f + m_jerk * dt * dt * dt / 6.0f;
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

    // Get time at starting step: t=0 for step 0, binary search otherwise
    float t = (startStep == 0) ? 0.0f : timeAtStep(startStep);
    float fTickFreq = (float)tickFreq;

    for (int32_t i = 0; i < toCompute; i++) {
        float v = velocityAt(t);
        if (v < MIN_VEL) v = MIN_VEL;

        float interval = fTickFreq / v;
        if (interval > 65535.0f) lut[i] = 65535;
        else if (interval < 1.0f) lut[i] = 1;
        else lut[i] = (uint16_t)(interval + 0.5f);

        // Advance time by one step (dt = 1/velocity)
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
