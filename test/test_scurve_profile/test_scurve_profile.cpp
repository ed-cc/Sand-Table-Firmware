// test_scurve_profile.cpp - Unit tests for SCurveProfile
#include <unity.h>
#include <Arduino.h>
#include <stdio.h>
#include "scurve_profile.h"
#include <math.h>

// Mock globals
unsigned long mock_micros_value = 0;
long mock_step_count = 0;
MockSerial Serial;

// Tolerances
static const float VEL_TOL = 5.0f;    // steps/sec tolerance for velocity checks
static const float TIME_TOL = 0.01f;   // seconds tolerance

// ===== Test Cases =====

void test_zero_steps_produces_no_profile() {
    SCurveProfile prof;
    prof.plan(0, 1000, 5000, 50000);
    TEST_ASSERT_EQUAL_INT32(0, prof.totalSteps());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, prof.totalTime());
}

void test_single_step_move() {
    SCurveProfile prof;
    prof.plan(1, 1000, 5000, 50000);
    TEST_ASSERT_EQUAL_INT32(1, prof.totalSteps());
    TEST_ASSERT_TRUE(prof.totalTime() > 0);
    // Should produce a valid interval
    uint16_t interval = prof.intervalAt(0, 16000000);
    TEST_ASSERT_TRUE(interval > 0);
    TEST_ASSERT_TRUE(interval <= 65535);
}

void test_long_move_has_all_seven_phases() {
    SCurveProfile prof;
    // Lots of steps, moderate speed/accel/jerk
    prof.plan(50000, 2000, 1500, 15000);

    // Phase durations: T1=T3=T5=T7 (jerk phases)
    float T1 = prof.phaseDuration(0);
    float T3 = prof.phaseDuration(2);
    float T5 = prof.phaseDuration(4);
    float T7 = prof.phaseDuration(6);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, T1, T3);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, T1, T5);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, T1, T7);

    // T2=T6 (const accel/decel phases)
    float T2 = prof.phaseDuration(1);
    float T6 = prof.phaseDuration(5);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, T2, T6);

    // T4 > 0 (cruise phase exists)
    float T4 = prof.phaseDuration(3);
    TEST_ASSERT_TRUE(T4 > 0);

    // All phases should have positive or zero duration
    for (int i = 0; i < 7; i++) {
        TEST_ASSERT_TRUE(prof.phaseDuration(i) >= 0);
    }
}

void test_velocity_starts_and_ends_at_zero() {
    SCurveProfile prof;
    prof.plan(10000, 2000, 1500, 15000);

    // Velocity at t=0 should be near zero (or MIN_VEL)
    float v0 = prof.velocityAt(0.0001f);
    TEST_ASSERT_TRUE(v0 < 50.0f);  // very low

    // Velocity at t=totalTime should be zero
    float vEnd = prof.velocityAt(prof.totalTime());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, vEnd);
}

void test_velocity_never_exceeds_max() {
    SCurveProfile prof;
    float maxVel = 2000;
    prof.plan(50000, maxVel, 1500, 15000);

    // Sample velocity at many points
    float dt = prof.totalTime() / 200.0f;
    for (int i = 0; i <= 200; i++) {
        float t = i * dt;
        float v = prof.velocityAt(t);
        // Allow small numerical overshoot
        TEST_ASSERT_TRUE(v <= maxVel + VEL_TOL);
    }
}

void test_velocity_reaches_near_max_on_long_move() {
    SCurveProfile prof;
    float maxVel = 2000;
    prof.plan(50000, maxVel, 1500, 15000);

    // Find peak velocity during cruise
    float cruiseStart = prof.phaseStartTime(3);
    float cruiseEnd = prof.phaseStartTime(4);
    float vCruise = prof.velocityAt((cruiseStart + cruiseEnd) / 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(VEL_TOL, maxVel, vCruise);
}

void test_short_move_no_cruise_phase() {
    SCurveProfile prof;
    // Very few steps - won't reach Vmax
    prof.plan(100, 2000, 1500, 15000);

    // Cruise phase should be zero or very small
    float T4 = prof.phaseDuration(3);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, T4);

    // Profile should still complete
    TEST_ASSERT_EQUAL_INT32(100, prof.totalSteps());
    TEST_ASSERT_TRUE(prof.totalTime() > 0);
}

void test_lut_intervals_are_valid() {
    SCurveProfile prof;
    prof.plan(1000, 2000, 1500, 15000);

    uint16_t lut[1000];
    int32_t count = prof.precomputeLUT(lut, 1000, 16000000);
    TEST_ASSERT_EQUAL_INT32(1000, count);

    // All intervals should be > 0
    for (int32_t i = 0; i < count; i++) {
        TEST_ASSERT_TRUE(lut[i] > 0);
    }
}

void test_lut_accel_phase_intervals_decrease() {
    SCurveProfile prof;
    prof.plan(10000, 2000, 1500, 15000);

    uint16_t lut[10000];
    prof.precomputeLUT(lut, 10000, 16000000);

    // During acceleration, intervals should generally decrease
    // (velocity increases -> shorter intervals)
    // Check first 50 steps (deep in accel phase)
    int decreasing = 0;
    for (int i = 1; i < 50; i++) {
        if (lut[i] <= lut[i - 1]) decreasing++;
    }
    // At least 80% should be decreasing (allowing some flat spots)
    TEST_ASSERT_TRUE(decreasing > 35);
}

void test_lut_decel_phase_intervals_increase() {
    SCurveProfile prof;
    prof.plan(10000, 2000, 1500, 15000);

    uint16_t lut[10000];
    prof.precomputeLUT(lut, 10000, 16000000);

    // During deceleration (last ~50 steps), intervals should generally increase
    int increasing = 0;
    for (int i = 9951; i < 10000; i++) {
        if (lut[i] >= lut[i - 1]) increasing++;
    }
    TEST_ASSERT_TRUE(increasing > 35);
}

void test_lut_max_entries_limit() {
    SCurveProfile prof;
    prof.plan(10000, 2000, 1500, 15000);

    uint16_t lut[100];
    int32_t count = prof.precomputeLUT(lut, 100, 16000000);
    TEST_ASSERT_EQUAL_INT32(100, count);
}

void test_is_complete() {
    SCurveProfile prof;
    prof.plan(1000, 2000, 1500, 15000);

    TEST_ASSERT_FALSE(prof.isComplete(0));
    TEST_ASSERT_FALSE(prof.isComplete(500));
    TEST_ASSERT_FALSE(prof.isComplete(999));
    TEST_ASSERT_TRUE(prof.isComplete(1000));
    TEST_ASSERT_TRUE(prof.isComplete(1001));
}

void test_symmetric_profile() {
    SCurveProfile prof;
    prof.plan(50000, 2000, 1500, 15000);

    // Velocity profile should be symmetric around midpoint
    float midTime = prof.totalTime() / 2.0f;
    float dt = prof.totalTime() / 100.0f;

    // Compare velocity at symmetric time points
    for (int i = 1; i < 20; i++) {
        float t1 = i * dt;
        float t2 = prof.totalTime() - i * dt;
        float v1 = prof.velocityAt(t1);
        float v2 = prof.velocityAt(t2);
        // Should be approximately equal (symmetric)
        float diff = fabsf(v1 - v2);
        float maxV = (v1 > v2) ? v1 : v2;
        if (maxV > 10.0f) {
            TEST_ASSERT_TRUE(diff / maxV < 0.05f);  // within 5%
        }
    }
}

void test_typical_radius_move() {
    // Simulate a real radius move: 100mm at system settings
    // 100mm * 125 steps/mm = 12500 steps
    SCurveProfile prof;
    prof.plan(12500, 20000, 10000, 100000);

    TEST_ASSERT_EQUAL_INT32(12500, prof.totalSteps());
    TEST_ASSERT_TRUE(prof.totalTime() > 0);

    // Check velocity doesn't exceed max
    float dt = prof.totalTime() / 100.0f;
    for (int i = 0; i <= 100; i++) {
        float v = prof.velocityAt(i * dt);
        TEST_ASSERT_TRUE(v <= 20005.0f);
    }
}

void test_typical_theta_move() {
    // Simulate a real theta move: 90 degrees
    // 90 * 116.667 = 10500 steps
    SCurveProfile prof;
    prof.plan(10500, 2000, 1500, 15000);

    TEST_ASSERT_EQUAL_INT32(10500, prof.totalSteps());
    TEST_ASSERT_TRUE(prof.totalTime() > 0);
}

// ===== Test Runner =====

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_steps_produces_no_profile);
    RUN_TEST(test_single_step_move);
    RUN_TEST(test_long_move_has_all_seven_phases);
    RUN_TEST(test_velocity_starts_and_ends_at_zero);
    RUN_TEST(test_velocity_never_exceeds_max);
    RUN_TEST(test_velocity_reaches_near_max_on_long_move);
    RUN_TEST(test_short_move_no_cruise_phase);
    RUN_TEST(test_lut_intervals_are_valid);
    RUN_TEST(test_lut_accel_phase_intervals_decrease);
    RUN_TEST(test_lut_decel_phase_intervals_increase);
    RUN_TEST(test_lut_max_entries_limit);
    RUN_TEST(test_is_complete);
    RUN_TEST(test_symmetric_profile);
    RUN_TEST(test_typical_radius_move);
    RUN_TEST(test_typical_theta_move);
    return UNITY_END();
}
