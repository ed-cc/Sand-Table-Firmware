// test_bresenham_sync.cpp - Unit tests for BresenhamSync
#include <unity.h>
#include <Arduino.h>
#include "bresenham_sync.h"

// Mock globals
unsigned long mock_micros_value = 0;
long mock_step_count = 0;
MockSerial Serial;

// Helper: run full Bresenham sequence and count minor steps
static int32_t runAndCountMinor(BresenhamSync& bres) {
    int32_t minorCount = 0;
    while (!bres.isComplete()) {
        if (bres.stepMinor()) minorCount++;
    }
    return minorCount;
}

// ===== Test Cases =====

void test_equal_axes() {
    BresenhamSync bres;
    bres.init(100, 100);
    // Minor fires every tick when both axes have equal steps
    int32_t minorCount = runAndCountMinor(bres);
    TEST_ASSERT_EQUAL_INT32(100, minorCount);
}

void test_2_to_1_ratio() {
    BresenhamSync bres;
    bres.init(100, 50);
    int32_t minorCount = runAndCountMinor(bres);
    TEST_ASSERT_EQUAL_INT32(50, minorCount);
}

void test_single_axis_a_only() {
    BresenhamSync bres;
    bres.init(100, 0);
    int32_t minorCount = runAndCountMinor(bres);
    TEST_ASSERT_EQUAL_INT32(0, minorCount);
    TEST_ASSERT_TRUE(bres.isADominant());
}

void test_single_axis_b_only() {
    BresenhamSync bres;
    bres.init(0, 100);
    int32_t minorCount = runAndCountMinor(bres);
    TEST_ASSERT_EQUAL_INT32(0, minorCount);
    TEST_ASSERT_FALSE(bres.isADominant());
}

void test_3_to_2_ratio() {
    BresenhamSync bres;
    bres.init(300, 200);
    int32_t minorCount = runAndCountMinor(bres);
    TEST_ASSERT_EQUAL_INT32(200, minorCount);
}

void test_large_step_counts() {
    BresenhamSync bres;
    bres.init(10000, 7777);
    int32_t minorCount = runAndCountMinor(bres);
    TEST_ASSERT_EQUAL_INT32(7777, minorCount);
}

void test_b_dominant() {
    BresenhamSync bres;
    bres.init(30, 100);
    TEST_ASSERT_FALSE(bres.isADominant());
    TEST_ASSERT_EQUAL_INT32(100, bres.totalDominant());
    TEST_ASSERT_EQUAL_INT32(30, bres.totalMinor());
    int32_t minorCount = runAndCountMinor(bres);
    TEST_ASSERT_EQUAL_INT32(30, minorCount);
}

void test_dominant_remaining_decrements() {
    BresenhamSync bres;
    bres.init(10, 5);
    TEST_ASSERT_EQUAL_INT32(10, bres.dominantRemaining());
    bres.stepMinor();
    TEST_ASSERT_EQUAL_INT32(9, bres.dominantRemaining());
    bres.stepMinor();
    TEST_ASSERT_EQUAL_INT32(8, bres.dominantRemaining());
}

void test_is_complete_transitions() {
    BresenhamSync bres;
    bres.init(3, 1);
    TEST_ASSERT_FALSE(bres.isComplete());
    bres.stepMinor(); // 2 remaining
    TEST_ASSERT_FALSE(bres.isComplete());
    bres.stepMinor(); // 1 remaining
    TEST_ASSERT_FALSE(bres.isComplete());
    bres.stepMinor(); // 0 remaining
    TEST_ASSERT_TRUE(bres.isComplete());
}

void test_step_distribution_evenness() {
    // For 100:60 ratio, steps should be distributed as evenly as possible
    BresenhamSync bres;
    bres.init(100, 60);

    int32_t gaps[60];
    int32_t gapIdx = 0;
    int32_t lastMinorAt = -1;
    int32_t domStep = 0;

    while (!bres.isComplete()) {
        bool fired = bres.stepMinor();
        if (fired && gapIdx < 60) {
            gaps[gapIdx++] = domStep - lastMinorAt;
            lastMinorAt = domStep;
        }
        domStep++;
    }

    // For 100:60, ideal spacing is 100/60 = 1.67, so gaps should be 1 or 2
    for (int32_t i = 1; i < gapIdx; i++) {  // skip first gap (includes initial offset)
        TEST_ASSERT_TRUE(gaps[i] >= 1);
        TEST_ASSERT_TRUE(gaps[i] <= 2);
    }
}

void test_zero_both_axes() {
    BresenhamSync bres;
    bres.init(0, 0);
    TEST_ASSERT_TRUE(bres.isComplete());
    TEST_ASSERT_EQUAL_INT32(0, bres.dominantRemaining());
}

void test_negative_values_treated_as_absolute() {
    // Bresenham uses absolute step counts
    BresenhamSync bres;
    bres.init(-100, -50);
    int32_t minorCount = runAndCountMinor(bres);
    TEST_ASSERT_EQUAL_INT32(50, minorCount);
}

void test_theta_compensation_realistic() {
    // Simulate: 90 deg theta move with compensation
    // theta steps = 10500, radius compensation steps = 5000
    BresenhamSync bres;
    bres.init(10500, 5000);
    TEST_ASSERT_TRUE(bres.isADominant());
    int32_t minorCount = runAndCountMinor(bres);
    TEST_ASSERT_EQUAL_INT32(5000, minorCount);
}

// ===== Test Runner =====

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_equal_axes);
    RUN_TEST(test_2_to_1_ratio);
    RUN_TEST(test_single_axis_a_only);
    RUN_TEST(test_single_axis_b_only);
    RUN_TEST(test_3_to_2_ratio);
    RUN_TEST(test_large_step_counts);
    RUN_TEST(test_b_dominant);
    RUN_TEST(test_dominant_remaining_decrements);
    RUN_TEST(test_is_complete_transitions);
    RUN_TEST(test_step_distribution_evenness);
    RUN_TEST(test_zero_both_axes);
    RUN_TEST(test_negative_values_treated_as_absolute);
    RUN_TEST(test_theta_compensation_realistic);
    return UNITY_END();
}
