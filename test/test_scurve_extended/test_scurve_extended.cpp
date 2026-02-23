// test_scurve_extended.cpp - Unit tests for SCurveProfile with boundary velocities
//
// Tests the asymmetric S-curve solver (v_start, v_end), planPhysical(),
// and steps_to_stop(). Also verifies backward compatibility with v_start=v_end=0.

#include <unity.h>
#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include "scurve_profile.h"
#include "values.h"

// Mock globals
unsigned long mock_micros_value = 0;
long mock_step_count = 0;
MockSerial Serial;

static const uint32_t TICK_FREQ = 16000000; // 16 MHz

static void assertFloatNear(float expected, float actual, float tol, const char* msg) {
    float diff = actual - expected;
    if (diff < 0) diff = -diff;
    if (diff > tol) {
        char buf[160];
        snprintf(buf, sizeof(buf), "%s: expected %.4f +/- %.4f, got %.4f",
                 msg, (double)expected, (double)tol, (double)actual);
        TEST_FAIL_MESSAGE(buf);
    }
}

// ---------------------------------------------------------------------------
// Case J: Accel-cruise-decel with non-zero boundary velocities
// ---------------------------------------------------------------------------
void test_case_J_nonzero_boundaries() {
    // 50mm move at 100 steps/mm = 5000 steps
    // v_start=20mm/s=2000 sps, v_nom=50mm/s=5000 sps, v_end=30mm/s=3000 sps
    // a_max=80mm/s^2=8000 sps^2, jerk=800mm/s^3=80000 sps^3
    SCurveProfile prof;
    float steps_per_mm = 100.0f;
    prof.planPhysical(50.0f, 20.0f, 50.0f, 30.0f, 80.0f, 800.0f, steps_per_mm);

    // Total steps should be 5000
    TEST_ASSERT_EQUAL_INT32(5000, prof.totalSteps());

    // Start velocity should be near v_start * steps_per_mm = 2000 sps
    float v_at_start = prof.velocityAt(0.0001f);
    // At t~0, velocity should be very close to v_start
    assertFloatNear(2000.0f, v_at_start, 50.0f, "J: velocity near start ≈ v_start");

    // Peak velocity should be v_nominal = 5000 sps
    assertFloatNear(5000.0f, prof.peakVelocity(), 50.0f, "J: peak ≈ v_nominal");

    // End velocity should be near v_end * steps_per_mm = 3000 sps
    float v_near_end = prof.velocityAt(prof.totalTime() - 0.0001f);
    assertFloatNear(3000.0f, v_near_end, 50.0f, "J: velocity near end ≈ v_end");

    // Verify stored boundary velocities
    assertFloatNear(2000.0f, prof.startVelocity(), 1.0f, "J: startVelocity()");
    assertFloatNear(3000.0f, prof.endVelocity(), 1.0f, "J: endVelocity()");

    // Verify LUT: first interval should correspond to ~v_start
    uint16_t lut[5000];
    int32_t entries = prof.precomputeLUT(lut, 5000, TICK_FREQ);
    TEST_ASSERT_EQUAL_INT32(5000, entries);

    // First interval: TICK_FREQ / v_start_sps = 16000000 / 2000 = 8000 ticks
    assertFloatNear(8000.0f, (float)lut[0], 200.0f, "J: first LUT interval ≈ v_start");

    // Last interval: TICK_FREQ / v_end_sps = 16000000 / 3000 = 5333 ticks
    assertFloatNear(5333.0f, (float)lut[entries - 1], 200.0f, "J: last LUT interval ≈ v_end");

    // Velocity profile should be smooth — no huge jumps between consecutive intervals
    for (int32_t i = 1; i < entries; i++) {
        float ratio = (float)lut[i] / (float)lut[i - 1];
        // Intervals shouldn't change by more than 5% per step
        TEST_ASSERT_TRUE(ratio > 0.90f && ratio < 1.10f);
    }
}

// ---------------------------------------------------------------------------
// Case K: Triangular profile (too short for cruise)
// ---------------------------------------------------------------------------
void test_case_K_triangular() {
    // 2mm move = 200 steps, v_start=0, v_nom=5000 sps, v_end=0
    // With a_max=8000 sps^2, jerk=80000 sps^3
    // This is too short to reach v_nom -> triangular
    SCurveProfile prof;
    prof.plan(200, 5000.0f, 8000.0f, 80000.0f, 0.0f, 0.0f);

    TEST_ASSERT_EQUAL_INT32(200, prof.totalSteps());

    // No cruise phase (T[3] = 0)
    assertFloatNear(0.0f, prof.phaseDuration(3), 0.001f, "K: no cruise phase");

    // Peak velocity should be well below nominal
    TEST_ASSERT_TRUE(prof.peakVelocity() < 5000.0f);
    TEST_ASSERT_TRUE(prof.peakVelocity() > 0.0f);

    // Start and end at 0
    assertFloatNear(0.0f, prof.startVelocity(), 0.01f, "K: start = 0");
    assertFloatNear(0.0f, prof.endVelocity(), 0.01f, "K: end = 0");

    // Verify LUT is 200 entries
    uint16_t lut[200];
    int32_t entries = prof.precomputeLUT(lut, 200, TICK_FREQ);
    TEST_ASSERT_EQUAL_INT32(200, entries);
}

// ---------------------------------------------------------------------------
// Case L: Pure cruise (v_start = v_end = v_nominal)
// ---------------------------------------------------------------------------
void test_case_L_pure_cruise() {
    // All boundary velocities equal to nominal -> pure cruise, no ramps
    SCurveProfile prof;
    prof.plan(1000, 5000.0f, 8000.0f, 80000.0f, 5000.0f, 5000.0f);

    TEST_ASSERT_EQUAL_INT32(1000, prof.totalSteps());

    // All phases except cruise should be zero (no accel or decel needed)
    assertFloatNear(0.0f, prof.phaseDuration(0), 0.001f, "L: no accel jerk-up");
    assertFloatNear(0.0f, prof.phaseDuration(1), 0.001f, "L: no const-accel");
    assertFloatNear(0.0f, prof.phaseDuration(2), 0.001f, "L: no accel jerk-down");
    // Cruise phase should cover all distance
    TEST_ASSERT_TRUE(prof.phaseDuration(3) > 0.0f);
    assertFloatNear(0.0f, prof.phaseDuration(4), 0.001f, "L: no decel jerk-down");
    assertFloatNear(0.0f, prof.phaseDuration(5), 0.001f, "L: no const-decel");
    assertFloatNear(0.0f, prof.phaseDuration(6), 0.001f, "L: no decel jerk-up");

    // All LUT intervals should be identical
    uint16_t lut[1000];
    int32_t entries = prof.precomputeLUT(lut, 1000, TICK_FREQ);
    TEST_ASSERT_EQUAL_INT32(1000, entries);

    uint16_t expected_interval = (uint16_t)(TICK_FREQ / 5000.0f + 0.5f); // 3200 ticks
    for (int32_t i = 0; i < entries; i++) {
        assertFloatNear((float)expected_interval, (float)lut[i], 2.0f, "L: uniform interval");
    }
}

// ---------------------------------------------------------------------------
// Asymmetric: v_start > v_end (mostly decelerating)
// ---------------------------------------------------------------------------
void test_asymmetric_decel_dominant() {
    // High entry speed, low exit speed over sufficient distance for full decel
    // v_start=4000, v_end=500, need enough steps for deceleration ramp
    SCurveProfile prof;
    prof.plan(5000, 5000.0f, 8000.0f, 80000.0f, 4000.0f, 500.0f);

    TEST_ASSERT_EQUAL_INT32(5000, prof.totalSteps());
    assertFloatNear(4000.0f, prof.startVelocity(), 1.0f, "asym: start = 4000");
    assertFloatNear(500.0f, prof.endVelocity(), 1.0f, "asym: end = 500");

    // Peak should be between start and nominal
    TEST_ASSERT_TRUE(prof.peakVelocity() >= 4000.0f - 1.0f);
    TEST_ASSERT_TRUE(prof.peakVelocity() <= 5000.0f + 1.0f);

    // First interval should be near TICK_FREQ/4000 = 4000 ticks
    uint16_t lut[5000];
    prof.precomputeLUT(lut, 5000, TICK_FREQ);
    float expected_first = TICK_FREQ / 4000.0f;
    assertFloatNear(expected_first, (float)lut[0], 200.0f, "asym: first interval");

    // Last interval should be near TICK_FREQ/500 = 32000 ticks
    float expected_last = TICK_FREQ / 500.0f;
    assertFloatNear(expected_last, (float)lut[4999], 1000.0f, "asym: last interval");
}

// ---------------------------------------------------------------------------
// steps_to_stop
// ---------------------------------------------------------------------------
void test_steps_to_stop() {
    // v=50mm/s, a_max=80mm/s^2, steps_per_mm=100
    // d = v^2/(2a) = 2500/160 = 15.625 mm = 1563 steps (rounded up)
    uint32_t steps = SCurveProfile::steps_to_stop(50.0f, 80.0f, 100.0f);
    // 15.625 * 100 + 1 = 1563.5 -> 1563
    TEST_ASSERT_TRUE(steps >= 1562 && steps <= 1564);

    // Zero velocity -> 0 steps
    TEST_ASSERT_EQUAL_UINT32(0, SCurveProfile::steps_to_stop(0.0f, 80.0f, 100.0f));
}

// ---------------------------------------------------------------------------
// planPhysical wrapper
// ---------------------------------------------------------------------------
void test_plan_physical() {
    SCurveProfile prof;
    // 10mm at 100 steps/mm = 1000 steps
    prof.planPhysical(10.0f, 0.0f, 50.0f, 0.0f, 80.0f, 800.0f, 100.0f);
    TEST_ASSERT_EQUAL_INT32(1000, prof.totalSteps());
    assertFloatNear(0.0f, prof.startVelocity(), 0.01f, "phys: start = 0");
    assertFloatNear(0.0f, prof.endVelocity(), 0.01f, "phys: end = 0");
}

// ---------------------------------------------------------------------------
// Backward compatibility: v_start=0, v_end=0 (default args)
// ---------------------------------------------------------------------------
void test_backward_compat_default_args() {
    SCurveProfile prof1, prof2;

    // Old-style call (4 args, defaults kick in)
    prof1.plan(1000, 5000.0f, 8000.0f, 80000.0f);

    // Explicit v_start=0, v_end=0
    prof2.plan(1000, 5000.0f, 8000.0f, 80000.0f, 0.0f, 0.0f);

    // Should produce identical profiles
    TEST_ASSERT_EQUAL_INT32(prof1.totalSteps(), prof2.totalSteps());
    assertFloatNear(prof1.totalTime(), prof2.totalTime(), 0.0001f, "compat: totalTime");
    assertFloatNear(prof1.peakVelocity(), prof2.peakVelocity(), 0.01f, "compat: peakVelocity");

    for (uint8_t i = 0; i < 7; i++) {
        assertFloatNear(prof1.phaseDuration(i), prof2.phaseDuration(i), 0.0001f,
                        "compat: phase duration");
    }
}

// ---------------------------------------------------------------------------
// Distance integrity check: numerically integrate and verify total distance
// ---------------------------------------------------------------------------
void test_distance_integrity_asymmetric() {
    SCurveProfile prof;
    prof.plan(3000, 5000.0f, 8000.0f, 80000.0f, 2000.0f, 1000.0f);

    // The LUT should have exactly totalSteps entries
    uint16_t lut[3000];
    int32_t entries = prof.precomputeLUT(lut, 3000, TICK_FREQ);
    TEST_ASSERT_EQUAL_INT32(3000, entries);

    // Sum of all 1/v (time increments) should approximate total time
    float total_time_from_lut = 0.0f;
    for (int32_t i = 0; i < entries; i++) {
        total_time_from_lut += (float)lut[i] / (float)TICK_FREQ;
    }

    // Should be within 5% of profile's total time
    float ratio = total_time_from_lut / prof.totalTime();
    TEST_ASSERT_TRUE(ratio > 0.95f && ratio < 1.05f);
}

// ===== Test Runner =====

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_case_J_nonzero_boundaries);
    RUN_TEST(test_case_K_triangular);
    RUN_TEST(test_case_L_pure_cruise);
    RUN_TEST(test_asymmetric_decel_dominant);
    RUN_TEST(test_steps_to_stop);
    RUN_TEST(test_plan_physical);
    RUN_TEST(test_backward_compat_default_args);
    RUN_TEST(test_distance_integrity_asymmetric);

    return UNITY_END();
}
