// test_stepper_axis.cpp - Unit tests for StepperAxis
#include <unity.h>
#include "stepper_axis.h"

// Mock globals required by Arduino.h mock
unsigned long mock_micros_value = 0;
long mock_step_count = 0;
MockSerial Serial;

// ===== Test Cases =====

void test_initial_position_is_zero() {
    StepperAxis axis;
    TEST_ASSERT_EQUAL_INT32(0, axis.position());
}

void test_initial_direction_is_forward() {
    StepperAxis axis;
    TEST_ASSERT_EQUAL_INT8(1, axis.direction());
}

void test_step_increments_position_forward() {
    StepperAxis axis;
    axis.init(10, 11, 12);
    axis.setDirection(1);
    mock_step_count = 0;
    axis.step();
    TEST_ASSERT_EQUAL_INT32(1, axis.position());
}

void test_step_decrements_position_reverse() {
    StepperAxis axis;
    axis.init(10, 11, 12);
    axis.setDirection(-1);
    axis.step();
    TEST_ASSERT_EQUAL_INT32(-1, axis.position());
}

void test_multiple_steps_accumulate() {
    StepperAxis axis;
    axis.init(10, 11, 12);
    axis.setDirection(1);
    for (int i = 0; i < 100; i++) {
        axis.step();
    }
    TEST_ASSERT_EQUAL_INT32(100, axis.position());
}

void test_direction_change_mid_move() {
    StepperAxis axis;
    axis.init(10, 11, 12);
    axis.setDirection(1);
    for (int i = 0; i < 50; i++) axis.step();
    axis.setDirection(-1);
    for (int i = 0; i < 30; i++) axis.step();
    TEST_ASSERT_EQUAL_INT32(20, axis.position());
}

void test_set_position_overrides() {
    StepperAxis axis;
    axis.init(10, 11, 12);
    axis.setDirection(1);
    for (int i = 0; i < 10; i++) axis.step();
    axis.setPosition(500);
    TEST_ASSERT_EQUAL_INT32(500, axis.position());
}

void test_set_position_then_step() {
    StepperAxis axis;
    axis.init(10, 11, 12);
    axis.setPosition(1000);
    axis.setDirection(1);
    axis.step();
    TEST_ASSERT_EQUAL_INT32(1001, axis.position());
}

void test_advance_position_without_pulse() {
    StepperAxis axis;
    axis.init(10, 11, 12);
    axis.setDirection(1);
    axis.advancePosition();
    TEST_ASSERT_EQUAL_INT32(1, axis.position());
    axis.setDirection(-1);
    axis.advancePosition();
    TEST_ASSERT_EQUAL_INT32(0, axis.position());
}

void test_port_registers_set_after_init() {
    StepperAxis axis;
    // Before init, ports should be null
    TEST_ASSERT_NULL(axis.stepPort);
    TEST_ASSERT_NULL(axis.dirPort);

    axis.init(10, 11, 12);

    // After init, ports should be non-null (dummy in test mode)
    TEST_ASSERT_NOT_NULL(axis.stepPort);
    TEST_ASSERT_NOT_NULL(axis.dirPort);
    TEST_ASSERT_NOT_EQUAL(0, axis.stepMask);
    TEST_ASSERT_NOT_EQUAL(0, axis.dirMask);
}

void test_steps_remaining_default_zero() {
    StepperAxis axis;
    TEST_ASSERT_EQUAL_INT32(0, axis.stepsRemaining);
}

void test_steps_remaining_writable() {
    StepperAxis axis;
    axis.stepsRemaining = 1000;
    TEST_ASSERT_EQUAL_INT32(1000, axis.stepsRemaining);
    axis.stepsRemaining--;
    TEST_ASSERT_EQUAL_INT32(999, axis.stepsRemaining);
}

void test_large_position_values() {
    StepperAxis axis;
    axis.setPosition(2000000);
    TEST_ASSERT_EQUAL_INT32(2000000, axis.position());
    axis.setPosition(-2000000);
    TEST_ASSERT_EQUAL_INT32(-2000000, axis.position());
}

void test_step_fires_digital_write() {
    StepperAxis axis;
    axis.init(10, 11, 12);
    mock_step_count = 0;
    axis.step();
    // Mock digitalWrite increments mock_step_count on HIGH
    TEST_ASSERT_GREATER_THAN(0, mock_step_count);
}

// ===== Test Runner =====

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_position_is_zero);
    RUN_TEST(test_initial_direction_is_forward);
    RUN_TEST(test_step_increments_position_forward);
    RUN_TEST(test_step_decrements_position_reverse);
    RUN_TEST(test_multiple_steps_accumulate);
    RUN_TEST(test_direction_change_mid_move);
    RUN_TEST(test_set_position_overrides);
    RUN_TEST(test_set_position_then_step);
    RUN_TEST(test_advance_position_without_pulse);
    RUN_TEST(test_port_registers_set_after_init);
    RUN_TEST(test_steps_remaining_default_zero);
    RUN_TEST(test_steps_remaining_writable);
    RUN_TEST(test_large_position_values);
    RUN_TEST(test_step_fires_digital_write);
    return UNITY_END();
}
