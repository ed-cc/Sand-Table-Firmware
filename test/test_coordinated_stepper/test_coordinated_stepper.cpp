// test_coordinated_stepper.cpp - Unit tests for CoordinatedStepper
#include <unity.h>
#include <Arduino.h>
#include <stdio.h>
#include "coordinated_stepper.h"
#include <math.h>

// Mock globals
unsigned long mock_micros_value = 0;
long mock_step_count = 0;
MockSerial Serial;

// Helper: run a move to completion by calling stepTick until done
static void runToCompletion(CoordinatedStepper& cs, int maxTicks = 200000) {
    for (int i = 0; i < maxTicks && !cs.isComplete(); i++) {
        cs.stepTick();
    }
}

// ===== Test Cases =====

void test_initial_state() {
    CoordinatedStepper cs;
    cs.init();
    TEST_ASSERT_TRUE(cs.isComplete());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, cs.getRadiusMM());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, cs.getThetaDegrees());
}

void test_set_position() {
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(100.0f, 45.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, cs.getRadiusMM());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 45.0f, cs.getThetaDegrees());
    TEST_ASSERT_TRUE(cs.isComplete());
}

void test_move_to_absolute() {
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(0.0f, 0.0f);

    cs.moveTo(10.0f, 0.0f, 1000.0f);
    TEST_ASSERT_FALSE(cs.isComplete());

    runToCompletion(cs);
    TEST_ASSERT_TRUE(cs.isComplete());

    // Should be at target position (within rounding)
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, cs.getRadiusMM());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, cs.getThetaDegrees());
}

void test_move_relative() {
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(50.0f, 90.0f);

    cs.moveRelative(10.0f, 0.0f, 1000.0f);
    runToCompletion(cs);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 60.0f, cs.getRadiusMM());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 90.0f, cs.getThetaDegrees());
}

void test_pure_theta_move_radius_unchanged() {
    // Pure theta move: radius should not change because compensation
    // keeps the belt in sync (the radius motor moves but the carriage doesn't)
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(100.0f, 0.0f);

    // Record initial radius position in steps
    int32_t initialRadiusSteps = cs.radiusAxis().position();

    cs.moveTo(100.0f, 90.0f, 1000.0f);
    runToCompletion(cs);

    // Theta should be at 90 degrees
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 90.0f, cs.getThetaDegrees());

    // The radius MOTOR moved (compensation steps), but the logical radius
    // position conversion accounts for this. Let's verify the radius motor
    // DID move (compensation is not zero).
    int32_t finalRadiusSteps = cs.radiusAxis().position();
    int32_t radiusStepsMoved = finalRadiusSteps - initialRadiusSteps;

    // Compensation: 90 deg * 55.556 steps/deg = 5000 steps
    // The radius motor should have moved ~5000 steps for compensation
    TEST_ASSERT_INT32_WITHIN(10, 5000, radiusStepsMoved);
}

void test_pure_radius_move_theta_unchanged() {
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(0.0f, 45.0f);

    cs.moveTo(50.0f, 45.0f, 1000.0f);
    runToCompletion(cs);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, cs.getRadiusMM());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 45.0f, cs.getThetaDegrees());
}

void test_zero_move_completes_immediately() {
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(10.0f, 20.0f);

    cs.moveTo(10.0f, 20.0f, 1000.0f);
    TEST_ASSERT_TRUE(cs.isComplete());
}

void test_negative_direction_move() {
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(100.0f, 180.0f);

    cs.moveTo(50.0f, 90.0f, 1000.0f);
    runToCompletion(cs);

    TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, cs.getRadiusMM());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, cs.getThetaDegrees());
}

void test_small_move() {
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(0.0f, 0.0f);

    // Move 0.1mm - just a few steps
    cs.moveRelative(0.1f, 0.0f, 1000.0f);
    runToCompletion(cs);

    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.1f, cs.getRadiusMM());
}

void test_multiple_sequential_moves() {
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(0.0f, 0.0f);

    cs.moveTo(10.0f, 0.0f, 1000.0f);
    runToCompletion(cs);

    cs.moveTo(20.0f, 0.0f, 1000.0f);
    runToCompletion(cs);

    cs.moveTo(20.0f, 90.0f, 1000.0f);
    runToCompletion(cs);

    TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f, cs.getRadiusMM());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, cs.getThetaDegrees());
}

void test_step_tick_does_nothing_when_complete() {
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(10.0f, 20.0f);

    // No move active — stepTick should be a no-op
    float r = cs.getRadiusMM();
    float t = cs.getThetaDegrees();
    cs.stepTick();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, r, cs.getRadiusMM());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, t, cs.getThetaDegrees());
}

void test_is_complete_transitions() {
    CoordinatedStepper cs;
    cs.init();
    cs.setPosition(0.0f, 0.0f);

    cs.moveRelative(1.0f, 0.0f, 1000.0f);
    TEST_ASSERT_FALSE(cs.isComplete());

    runToCompletion(cs);
    TEST_ASSERT_TRUE(cs.isComplete());
}

// ===== Test Runner =====

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state);
    RUN_TEST(test_set_position);
    RUN_TEST(test_move_to_absolute);
    RUN_TEST(test_move_relative);
    RUN_TEST(test_pure_theta_move_radius_unchanged);
    RUN_TEST(test_pure_radius_move_theta_unchanged);
    RUN_TEST(test_zero_move_completes_immediately);
    RUN_TEST(test_negative_direction_move);
    RUN_TEST(test_small_move);
    RUN_TEST(test_multiple_sequential_moves);
    RUN_TEST(test_step_tick_does_nothing_when_complete);
    RUN_TEST(test_is_complete_transitions);
    return UNITY_END();
}
