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

// ===== Compressed LUT Tests =====

void test_lut_entry_count_is_compressed_for_long_move() {
    // Test the new compression API: lutEntryCount() should return
    // a much smaller count than totalSteps() for moves with long cruise phase.
    // Use system-realistic parameters for sand table: fast accel/jerk, slow speed
    SCurveProfile prof;
    // Large move (15000 steps) with system parameters:
    // maxVel=2083 (16.67mm/s at 125 steps/mm), high accel/jerk
    prof.plan(15000, 2083, 10000, 100000);

    TEST_ASSERT_EQUAL_INT32(15000, prof.totalSteps());

    // lutEntryCount() should be much smaller due to cruise compression
    int32_t compressedCount = prof.lutEntryCount();
    // With system parameters: ~150 accel + 4 cruise + ~150 decel = ~300 entries
    TEST_ASSERT_TRUE(compressedCount < prof.totalSteps());
    TEST_ASSERT_TRUE(compressedCount < 1400);  // Must fit in AVR_MAX_PRECOMPUTE_STEPS
}

void test_compressed_lut_smaller_than_step_count() {
    // Test precomputeCompressedLUT() returns fewer entries than totalSteps()
    // Use system-realistic parameters (fast accel/jerk)
    SCurveProfile prof;
    prof.plan(15000, 2083, 10000, 100000);  // system defaults: high accel/jerk, slower speed

    uint16_t lut[2000];
    int32_t compressedEntries = prof.precomputeCompressedLUT(lut, 2000, 16000000);

    // Compressed format should have far fewer entries than 15000 steps
    TEST_ASSERT_TRUE(compressedEntries < 15000);
    // With system params, should be ~300 entries vs 15000 steps
    TEST_ASSERT_TRUE(compressedEntries < 1000);  // Dramatic compression
    TEST_ASSERT_EQUAL_INT32(compressedEntries, prof.lutEntryCount());
}

void test_compressed_lut_cruise_header_has_correct_count() {
    // Test that the compressed LUT contains a cruise sentinel (0 value)
    // and the header has proper structure. Must skip over accel sentinels.
    SCurveProfile prof;
    prof.plan(15000, 2000, 1500, 15000);

    uint16_t lut[2000];
    int32_t compressedEntries = prof.precomputeCompressedLUT(lut, 2000, 16000000);

    // Find the cruise sentinel (value 0), skipping over accel sentinel data
    int32_t sentinelPos = -1;
    int32_t i = 0;
    while (i < compressedEntries) {
        if (lut[i] == 0) {
            sentinelPos = i;
            break;
        } else if (lut[i] == 1) {
            i += 7;  // skip accel sentinel header
        } else {
            i++;     // raw interval
        }
    }

    // Should find a cruise sentinel
    TEST_ASSERT_NOT_EQUAL(-1, sentinelPos);
    TEST_ASSERT_TRUE(sentinelPos > 0);  // not at the very start

    // After sentinel, next three entries should be: cruise_interval, count_hi, count_lo
    TEST_ASSERT_TRUE(sentinelPos + 3 < compressedEntries);
    uint16_t cruiseInterval = lut[sentinelPos + 1];
    uint16_t countHi = lut[sentinelPos + 2];
    uint16_t countLo = lut[sentinelPos + 3];

    // Cruise interval should be non-zero
    TEST_ASSERT_TRUE(cruiseInterval > 0);

    // Reconstruct cruise step count
    uint32_t cruiseCount = ((uint32_t)countHi << 16) | countLo;

    // Cruise count should be reasonable (positive, large for this move)
    TEST_ASSERT_TRUE(cruiseCount > 0);
    TEST_ASSERT_TRUE(cruiseCount < 15000);
}

// ===== Phase Boundaries Tests =====

void test_phase_boundaries_sum_equals_total_steps() {
    SCurveProfile prof;
    prof.plan(50000, 2000, 1500, 15000);

    SCurveProfile::PhaseBoundaries pb;
    prof.phaseBoundaries(pb);

    int32_t sum = pb.jerk0Steps + pb.constAccelSteps + pb.jerk2Steps
                + pb.cruiseSteps + pb.jerk4Steps + pb.constDecelSteps + pb.jerk6Steps;
    // Allow rounding tolerance of ±1
    TEST_ASSERT_INT32_WITHIN(1, prof.totalSteps(), sum);
}

void test_phase_boundaries_long_move_has_const_accel() {
    SCurveProfile prof;
    prof.plan(50000, 2000, 1500, 15000);

    SCurveProfile::PhaseBoundaries pb;
    prof.phaseBoundaries(pb);

    TEST_ASSERT_TRUE(pb.constAccelSteps > 0);
    TEST_ASSERT_TRUE(pb.constDecelSteps > 0);
    TEST_ASSERT_TRUE(pb.cruiseSteps > 0);
}

void test_phase_boundaries_short_move_no_const_accel() {
    SCurveProfile prof;
    // Very short move: triangular profile, no const-accel or cruise
    prof.plan(10, 2000, 1500, 15000);

    SCurveProfile::PhaseBoundaries pb;
    prof.phaseBoundaries(pb);

    // For such a short move, const accel and cruise should be 0
    TEST_ASSERT_EQUAL_INT32(0, pb.constAccelSteps);
    TEST_ASSERT_EQUAL_INT32(0, pb.cruiseSteps);
    TEST_ASSERT_EQUAL_INT32(0, pb.constDecelSteps);
}

void test_phase_boundaries_zero_steps() {
    SCurveProfile prof;
    prof.plan(0, 2000, 1500, 15000);

    SCurveProfile::PhaseBoundaries pb;
    prof.phaseBoundaries(pb);

    TEST_ASSERT_EQUAL_INT32(0, pb.jerk0Steps);
    TEST_ASSERT_EQUAL_INT32(0, pb.constAccelSteps);
    TEST_ASSERT_EQUAL_INT32(0, pb.jerk2Steps);
    TEST_ASSERT_EQUAL_INT32(0, pb.cruiseSteps);
    TEST_ASSERT_EQUAL_INT32(0, pb.jerk4Steps);
    TEST_ASSERT_EQUAL_INT32(0, pb.constDecelSteps);
    TEST_ASSERT_EQUAL_INT32(0, pb.jerk6Steps);
}

void test_phase_boundaries_consistent_with_lut_entry_count() {
    SCurveProfile prof;
    prof.plan(15000, 2083, 10000, 100000);

    SCurveProfile::PhaseBoundaries pb;
    prof.phaseBoundaries(pb);
    int32_t lutCount = prof.lutEntryCount();

    // Manual expected count from boundaries
    int32_t expected = pb.jerk0Steps
                     + (pb.constAccelSteps > 0 ? 7 : 0)
                     + pb.jerk2Steps
                     + (pb.cruiseSteps > 0 ? 4 : 0)
                     + pb.jerk4Steps
                     + (pb.constDecelSteps > 0 ? 7 : 0)
                     + pb.jerk6Steps;

    TEST_ASSERT_EQUAL_INT32(expected, lutCount);
}

// ===== Accel Sentinel Tests =====

void test_compressed_lut_contains_accel_sentinel() {
    // A long move with const-accel phases should contain sentinel value 1
    SCurveProfile prof;
    prof.plan(50000, 2000, 1500, 15000);

    SCurveProfile::PhaseBoundaries pb;
    prof.phaseBoundaries(pb);

    // Ensure const-accel phases exist
    TEST_ASSERT_TRUE(pb.constAccelSteps > 0);

    uint16_t lut[2000];
    int32_t entries = prof.precomputeCompressedLUT(lut, 2000, 16000000);

    // Find accel sentinel (value 1)
    int32_t accelSentinelPos = -1;
    for (int32_t i = 0; i < entries; i++) {
        if (lut[i] == 1) {
            accelSentinelPos = i;
            break;
        }
    }
    TEST_ASSERT_NOT_EQUAL(-1, accelSentinelPos);
    TEST_ASSERT_TRUE(accelSentinelPos + 6 < entries);  // room for 7-entry header
}

void test_accel_sentinel_velocity_matches_profile() {
    SCurveProfile prof;
    prof.plan(50000, 2000, 1500, 15000);

    uint16_t lut[2000];
    int32_t entries = prof.precomputeCompressedLUT(lut, 2000, 16000000);

    // Find first accel sentinel
    for (int32_t i = 0; i < entries; i++) {
        if (lut[i] == 1) {
            // Extract velocity from sentinel
            uint32_t v_fixed = ((uint32_t)lut[i+1] << 16) | lut[i+2];
            float v_from_sentinel = (float)v_fixed / 65536.0f;

            // Should match V[1] (phase 1 start velocity)
            float v_expected = prof.velocityAt(prof.phaseStartTime(1) + 0.0001f);
            // Allow 5% tolerance due to fixed-point rounding
            float diff = fabsf(v_from_sentinel - v_expected);
            TEST_ASSERT_TRUE(diff / v_expected < 0.05f);
            break;
        }
    }
}

void test_compressed_lut_total_steps_correct() {
    // Verify that the sum of all sentinel counts + raw entries equals totalSteps
    SCurveProfile prof;
    prof.plan(15000, 2000, 1500, 15000);

    uint16_t lut[2000];
    int32_t entries = prof.precomputeCompressedLUT(lut, 2000, 16000000);

    int32_t totalStepsFromLut = 0;
    int32_t i = 0;
    while (i < entries) {
        if (lut[i] == 0) {
            // Cruise sentinel: [0, interval, count_hi, count_lo]
            uint32_t count = ((uint32_t)lut[i+2] << 16) | lut[i+3];
            totalStepsFromLut += (int32_t)count;
            i += 4;
        } else if (lut[i] == 1) {
            // Accel sentinel: [1, v_hi, v_lo, a_hi, a_lo, count_hi, count_lo]
            uint32_t count = ((uint32_t)lut[i+5] << 16) | lut[i+6];
            totalStepsFromLut += (int32_t)count;
            i += 7;
        } else {
            // Raw interval: one step
            totalStepsFromLut++;
            i++;
        }
    }

    // Allow ±1 for rounding
    TEST_ASSERT_INT32_WITHIN(1, prof.totalSteps(), totalStepsFromLut);
}

void test_lut_entry_count_much_smaller_with_accel_sentinels() {
    // With theta-like parameters that produce large const-accel phases,
    // the new compression should be dramatically smaller
    SCurveProfile prof;
    prof.plan(5250, 2000, 1500, 15000);  // typical 45-degree theta move

    int32_t compressed = prof.lutEntryCount();

    // With accel sentinels, should be well under 1400 (AVR_MAX_PRECOMPUTE_STEPS)
    TEST_ASSERT_TRUE(compressed < 1400);
    // Should be dramatically less than total steps
    TEST_ASSERT_TRUE(compressed < prof.totalSteps() / 2);
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
    RUN_TEST(test_lut_entry_count_is_compressed_for_long_move);
    RUN_TEST(test_compressed_lut_smaller_than_step_count);
    RUN_TEST(test_compressed_lut_cruise_header_has_correct_count);
    RUN_TEST(test_phase_boundaries_sum_equals_total_steps);
    RUN_TEST(test_phase_boundaries_long_move_has_const_accel);
    RUN_TEST(test_phase_boundaries_short_move_no_const_accel);
    RUN_TEST(test_phase_boundaries_zero_steps);
    RUN_TEST(test_phase_boundaries_consistent_with_lut_entry_count);
    RUN_TEST(test_compressed_lut_contains_accel_sentinel);
    RUN_TEST(test_accel_sentinel_velocity_matches_profile);
    RUN_TEST(test_compressed_lut_total_steps_correct);
    RUN_TEST(test_lut_entry_count_much_smaller_with_accel_sentinels);
    return UNITY_END();
}
