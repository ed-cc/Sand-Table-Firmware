// test_theta_compensation.cpp - Unit tests for ThetaCompensation
#include <unity.h>
#include <Arduino.h>
#include <stdio.h>
#include "theta_compensation.h"
#include "values.h"
#include <math.h>

// Mock globals required by Arduino.h mock
unsigned long mock_micros_value = 0;
long mock_step_count = 0;
MockSerial Serial;

// Helper: check step count is within +/- 1 of expected (rounding tolerance)
static void assertStepsNear(int32_t expected, int32_t actual, const char* msg) {
    if (abs(actual - expected) > 1) {
        char buf[128];
        snprintf(buf, sizeof(buf), "%s: expected %d +/-1, got %d", msg, (int)expected, (int)actual);
        TEST_FAIL_MESSAGE(buf);
    }
}

// ===== Test Cases =====

void test_pure_theta_90_degrees() {
    // Pure theta: delta_r = 0, delta_theta = 90
    // thetaSteps = round(90 * 116.667) = 10500
    // radiusSteps = round(0) + round(90 * 55.556) = 5000 (compensation only)
    MoveSteps steps = ThetaCompensation::decompose(0.0f, 90.0f);
    assertStepsNear(10500, steps.thetaSteps, "theta 90deg: thetaSteps");
    assertStepsNear(5000, steps.radiusSteps, "theta 90deg: radiusSteps (comp)");
}

void test_pure_radius_10mm() {
    // Pure radius: delta_r = 10mm, delta_theta = 0
    // thetaSteps = 0
    // radiusSteps = round(10 * 125) = 1250
    MoveSteps steps = ThetaCompensation::decompose(10.0f, 0.0f);
    TEST_ASSERT_EQUAL_INT32(0, steps.thetaSteps);
    assertStepsNear(1250, steps.radiusSteps, "radius 10mm: radiusSteps");
}

void test_combined_move() {
    // Combined: delta_r = 10mm, delta_theta = 90deg
    // thetaSteps = round(90 * 116.667) = 10500
    // radiusSteps = round(10 * 125) + round(90 * 55.556) = 1250 + 5000 = 6250
    MoveSteps steps = ThetaCompensation::decompose(10.0f, 90.0f);
    assertStepsNear(10500, steps.thetaSteps, "combined: thetaSteps");
    assertStepsNear(6250, steps.radiusSteps, "combined: radiusSteps");
}

void test_negative_theta() {
    // Negative theta: compensation reverses sign
    MoveSteps steps = ThetaCompensation::decompose(0.0f, -90.0f);
    assertStepsNear(-10500, steps.thetaSteps, "neg theta: thetaSteps");
    assertStepsNear(-5000, steps.radiusSteps, "neg theta: radiusSteps");
}

void test_negative_radius() {
    MoveSteps steps = ThetaCompensation::decompose(-10.0f, 0.0f);
    TEST_ASSERT_EQUAL_INT32(0, steps.thetaSteps);
    assertStepsNear(-1250, steps.radiusSteps, "neg radius: radiusSteps");
}

void test_zero_move() {
    MoveSteps steps = ThetaCompensation::decompose(0.0f, 0.0f);
    TEST_ASSERT_EQUAL_INT32(0, steps.thetaSteps);
    TEST_ASSERT_EQUAL_INT32(0, steps.radiusSteps);
}

void test_small_fractional_move() {
    // 0.01 degrees of theta
    // thetaSteps = round(0.01 * 116.667) = round(1.167) = 1
    // compensation = round(0.01 * 55.556) = round(0.556) = 1
    MoveSteps steps = ThetaCompensation::decompose(0.0f, 0.01f);
    assertStepsNear(1, steps.thetaSteps, "small: thetaSteps");
    assertStepsNear(1, steps.radiusSteps, "small: radiusSteps");
}

void test_full_revolution() {
    // 360 degrees
    // thetaSteps = round(360 * 116.667) = 42000
    // compensation = round(360 * 55.556) = 20000
    MoveSteps steps = ThetaCompensation::decompose(0.0f, 360.0f);
    assertStepsNear(42000, steps.thetaSteps, "full rev: thetaSteps");
    assertStepsNear(20000, steps.radiusSteps, "full rev: radiusSteps (comp)");
}

void test_compensation_ratio_matches_values_h() {
    // For 210 theta motor steps, compensation should be 100 radius steps.
    // 210 theta steps = 210 / 116.667 degrees = 1.8 degrees
    // compensation for 1.8 deg = 1.8 * 55.556 = 100.0 steps
    float thetaDeg = 210.0f / THETA_STEPS_PER_DEGREE;
    MoveSteps steps = ThetaCompensation::decompose(0.0f, thetaDeg);
    assertStepsNear(210, steps.thetaSteps, "ratio: thetaSteps");
    assertStepsNear(100, steps.radiusSteps, "ratio: radiusSteps");
}

void test_opposite_directions_cancel() {
    // Radius inward + theta positive: compensation partially cancels radius command
    // delta_r = -5mm -> -625 steps, delta_theta = 11.25 deg -> comp = 625 steps
    // 11.25 * 55.556 = 625
    MoveSteps steps = ThetaCompensation::decompose(-5.0f, 11.25f);
    assertStepsNear((int32_t)roundf(11.25f * THETA_STEPS_PER_DEGREE),
                    steps.thetaSteps, "cancel: thetaSteps");
    // radius: -625 + 625 = 0
    assertStepsNear(0, steps.radiusSteps, "cancel: radiusSteps");
}

// ===== Test Runner =====

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_pure_theta_90_degrees);
    RUN_TEST(test_pure_radius_10mm);
    RUN_TEST(test_combined_move);
    RUN_TEST(test_negative_theta);
    RUN_TEST(test_negative_radius);
    RUN_TEST(test_zero_move);
    RUN_TEST(test_small_fractional_move);
    RUN_TEST(test_full_revolution);
    RUN_TEST(test_compensation_ratio_matches_values_h);
    RUN_TEST(test_opposite_directions_cancel);
    return UNITY_END();
}
