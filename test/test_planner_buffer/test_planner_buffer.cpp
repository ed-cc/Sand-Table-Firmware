// test_planner_buffer.cpp - Unit tests for PlannerBuffer (Phases 1 and 2)
//
// Tests junction speed calculation, physical vector computation,
// two-pass lookahead, and collinear merge.

#include <unity.h>
#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include "planner_buffer.h"
#include "values.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Mock globals required by Arduino.h mock
unsigned long mock_micros_value = 0;
long mock_step_count = 0;
MockSerial Serial;

// Helper: assert float within tolerance
static void assertFloatNear(float expected, float actual, float tol, const char* msg) {
    float diff = actual - expected;
    if (diff < 0) diff = -diff;
    if (diff > tol) {
        char buf[128];
        snprintf(buf, sizeof(buf), "%s: expected %.4f +/- %.4f, got %.4f",
                 msg, (double)expected, (double)tol, (double)actual);
        TEST_FAIL_MESSAGE(buf);
    }
}

// ---------------------------------------------------------------------------
// Phase 1: Junction Speed Tests (Cases A-F)
// ---------------------------------------------------------------------------

// Build a PlannerBlock with physical vector computed, for junction speed testing.
static PlannerBlock make_block(float delta_r_mm, float delta_theta_deg,
                               float feed_mmps, float R_start_mm) {
    PlannerBlock blk;
    memset(&blk, 0, sizeof(blk));
    blk.delta_r_mm = delta_r_mm;
    blk.delta_theta_deg = delta_theta_deg;
    blk.nominal_speed_mmps = feed_mmps;
    blk.R_start_mm = R_start_mm;

    float arc_mm = 0.0f;
    if (R_start_mm >= PLANNER_R_MIN_MM) {
        float delta_theta_rad = delta_theta_deg * (float)M_PI / 180.0f;
        arc_mm = R_start_mm * delta_theta_rad;
    }
    blk.phys_distance_mm = sqrtf(delta_r_mm * delta_r_mm + arc_mm * arc_mm);
    if (blk.phys_distance_mm > 0.001f) {
        blk.unit_phys[0] = delta_r_mm / blk.phys_distance_mm;
        blk.unit_phys[1] = arc_mm / blk.phys_distance_mm;
    } else {
        blk.unit_phys[0] = 0.0f;
        blk.unit_phys[1] = 0.0f;
    }
    return blk;
}

void test_case_A_collinear_radius() {
    // Two consecutive (+10 mm, 0 deg) moves at 50 mm/s, R = 50 mm
    // Collinear -> junction speed = min(50, 50) = 50 mm/s
    PlannerBlock prev = make_block(10.0f, 0.0f, 50.0f, 50.0f);
    PlannerBlock curr = make_block(10.0f, 0.0f, 50.0f, 60.0f);

    float v_junc = compute_max_entry_speed(prev, curr,
                                           JUNCTION_DEVIATION_MM,
                                           DEFAULT_PHYS_ACCEL_MMPS2);
    assertFloatNear(50.0f, v_junc, 0.01f, "Case A: collinear junction speed");
}

void test_case_B_90deg_turn_R50() {
    // Move 1: (0 mm, +90 deg) at R = 50 mm -> arc = 50 * pi/2 ~ 78.5 mm
    //   unit = {0, 1}
    // Move 2: (+10 mm, 0 deg) -> unit = {1, 0}
    // cos_theta = -(0*1 + 1*0) = 0
    // sin(theta/2) = sqrt(0.5*(1-0)) = 1/sqrt(2) ~ 0.7071
    // denom = 1 - 0.7071 = 0.2929
    // R_jd = 0.05 * 0.7071 / 0.2929 ~ 0.1207
    // v_max = sqrt(0.1207 * 80) ~ sqrt(9.657) ~ 3.108
    // Wait - the accel is DEFAULT_PHYS_ACCEL_MMPS2 = 80 mm/s^2
    // v = sqrt(R_jd * accel) = sqrt(0.1207 * 80) = sqrt(9.656) = 3.107 mm/s
    //
    // But the report uses accel = 4000 mm/s^2:
    //   v = sqrt(0.1207 * 4000) = sqrt(482.8) = 21.97 mm/s
    // Our DEFAULT_PHYS_ACCEL_MMPS2 = 10000/125 = 80 mm/s^2
    // So with 80: v = sqrt(0.1207 * 80) ~ 3.11 mm/s
    PlannerBlock prev = make_block(0.0f, 90.0f, 50.0f, 50.0f);
    PlannerBlock curr = make_block(10.0f, 0.0f, 50.0f, 50.0f);

    float v_junc = compute_max_entry_speed(prev, curr,
                                           JUNCTION_DEVIATION_MM,
                                           DEFAULT_PHYS_ACCEL_MMPS2);

    // Compute expected analytically
    float cos_theta = 0.0f; // -(0*1 + 1*0) = 0
    float sin_d2 = sqrtf(0.5f * (1.0f - cos_theta)); // sqrt(0.5) = 0.7071
    float denom = 1.0f - sin_d2; // 0.2929
    float R_jd = JUNCTION_DEVIATION_MM * sin_d2 / denom;
    float expected = sqrtf(R_jd * DEFAULT_PHYS_ACCEL_MMPS2);

    assertFloatNear(expected, v_junc, 0.01f, "Case B: 90deg turn at R=50");
    // Also verify it's > 0 and < 50
    TEST_ASSERT_TRUE(v_junc > 0.0f);
    TEST_ASSERT_TRUE(v_junc < 50.0f);
}

void test_case_C_90deg_turn_R5() {
    // Same 90-degree turn at R = 5 mm
    // unit vectors should be identical: pure theta move -> {0, 1}, pure radius -> {1, 0}
    // Junction speed should be identical to Case B (angle-dependent only)
    PlannerBlock prev = make_block(0.0f, 90.0f, 50.0f, 5.0f);
    PlannerBlock curr = make_block(10.0f, 0.0f, 50.0f, 5.0f);

    float v_junc_R5 = compute_max_entry_speed(prev, curr,
                                               JUNCTION_DEVIATION_MM,
                                               DEFAULT_PHYS_ACCEL_MMPS2);

    // Compare with R = 50
    PlannerBlock prev50 = make_block(0.0f, 90.0f, 50.0f, 50.0f);
    PlannerBlock curr50 = make_block(10.0f, 0.0f, 50.0f, 50.0f);

    float v_junc_R50 = compute_max_entry_speed(prev50, curr50,
                                                JUNCTION_DEVIATION_MM,
                                                DEFAULT_PHYS_ACCEL_MMPS2);

    assertFloatNear(v_junc_R50, v_junc_R5, 0.01f,
                    "Case C: 90deg turn at R=5 same as R=50");
}

void test_case_D_reversal() {
    // Two opposite-direction radius moves: (+10, 0) then (-10, 0)
    // cos_theta = -(1*(-1) + 0*0) = 1 -> reversal -> speed = 0
    PlannerBlock prev = make_block(10.0f, 0.0f, 50.0f, 50.0f);
    PlannerBlock curr = make_block(-10.0f, 0.0f, 50.0f, 60.0f);

    float v_junc = compute_max_entry_speed(prev, curr,
                                           JUNCTION_DEVIATION_MM,
                                           DEFAULT_PHYS_ACCEL_MMPS2);
    assertFloatNear(0.0f, v_junc, 0.01f, "Case D: reversal = 0");
}

void test_case_E_mixed_move() {
    // Move 1: (+5 mm, +45 deg) at R = 40 mm
    //   arc = 40 * 45 * pi/180 = 40 * 0.7854 = 31.416 mm
    //   dist = sqrt(25 + 987.06) = sqrt(1012.06) = 31.81 mm
    //   unit = {5/31.81, 31.416/31.81} = {0.1572, 0.9876}
    // Move 2: (+5 mm, -45 deg) at R = 45 mm (R increased by 5)
    //   arc = 45 * (-45) * pi/180 = -35.34 mm
    //   dist = sqrt(25 + 1249.0) = sqrt(1274.0) = 35.69 mm
    //   unit = {5/35.69, -35.34/35.69} = {0.1401, -0.9901}
    // cos_theta = -(0.1572*0.1401 + 0.9876*(-0.9901))
    //           = -(0.02203 - 0.97783) = 0.9558
    // This is a near-reversal in theta with constant forward r -> very low speed
    PlannerBlock prev = make_block(5.0f, 45.0f, 50.0f, 40.0f);
    PlannerBlock curr = make_block(5.0f, -45.0f, 50.0f, 45.0f);

    float v_junc = compute_max_entry_speed(prev, curr,
                                           JUNCTION_DEVIATION_MM,
                                           DEFAULT_PHYS_ACCEL_MMPS2);

    // Should be very low (near 0) due to near-reversal
    TEST_ASSERT_TRUE(v_junc >= 0.0f);
    TEST_ASSERT_TRUE(v_junc < 5.0f); // well below nominal

    // Compute analytically to verify
    float arc1 = 40.0f * 45.0f * (float)M_PI / 180.0f;
    float dist1 = sqrtf(25.0f + arc1 * arc1);
    float u1_r = 5.0f / dist1;
    float u1_th = arc1 / dist1;

    float arc2 = 45.0f * (-45.0f) * (float)M_PI / 180.0f;
    float dist2 = sqrtf(25.0f + arc2 * arc2);
    float u2_r = 5.0f / dist2;
    float u2_th = arc2 / dist2;

    float cos_theta = -(u1_r * u2_r + u1_th * u2_th);
    float sin_d2 = sqrtf(0.5f * (1.0f - cos_theta));
    float denom = 1.0f - sin_d2;
    float expected = 0.0f;
    if (denom > 1e-5f) {
        float R_jd = JUNCTION_DEVIATION_MM * sin_d2 / denom;
        expected = sqrtf(R_jd * DEFAULT_PHYS_ACCEL_MMPS2);
        if (expected > 50.0f) expected = 50.0f;
    }

    assertFloatNear(expected, v_junc, 0.01f, "Case E: mixed move junction");
}

void test_case_F_below_R_min() {
    // R_start = 0.5 mm (below PLANNER_R_MIN_MM = 1.0)
    // When pushed via PlannerBuffer, max_entry_speed should be forced to 0.
    // Test the physical vector computation: arc_mm should be 0.
    PlannerBlock blk = make_block(0.0f, 90.0f, 50.0f, 0.5f);

    // With R < R_MIN, the make_block helper already sets arc_mm = 0
    // phys_distance should be 0 (no radius change, arc forced to 0)
    assertFloatNear(0.0f, blk.phys_distance_mm, 0.001f,
                    "Case F: phys_distance at R<R_min");

    // Junction speed should be 0 when a dwell is involved
    PlannerBlock prev = make_block(10.0f, 0.0f, 50.0f, 5.0f);
    float v_junc = compute_max_entry_speed(prev, blk,
                                           JUNCTION_DEVIATION_MM,
                                           DEFAULT_PHYS_ACCEL_MMPS2);
    assertFloatNear(0.0f, v_junc, 0.01f, "Case F: junction speed at R<R_min = 0");
}

void test_sizeof_planner_block() {
    TEST_ASSERT_TRUE(sizeof(PlannerBlock) <= 56);
}

// ---------------------------------------------------------------------------
// Phase 1: PlannerBuffer push() integration tests
// ---------------------------------------------------------------------------

void test_push_single_block() {
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    buf.push(10.0f, 0.0f, 50.0f, 50.0f);
    TEST_ASSERT_EQUAL_UINT8(1, buf.count());
    TEST_ASSERT_FALSE(buf.empty());
    TEST_ASSERT_FALSE(buf.full());

    PlannerBlock* blk = buf.current();
    TEST_ASSERT_NOT_NULL(blk);
    assertFloatNear(10.0f, blk->phys_distance_mm, 0.01f, "push: phys_distance");
    assertFloatNear(0.0f, blk->entry_speed_mmps, 0.01f, "push: entry = 0 (first block)");
}

void test_push_two_collinear_merge() {
    // Two collinear same-speed blocks should merge into one
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    buf.push(10.0f, 0.0f, 50.0f, 50.0f);
    buf.push(10.0f, 0.0f, 50.0f, 60.0f);

    // Collinear + same feed rate -> merged into 1 block
    TEST_ASSERT_EQUAL_UINT8(1, buf.count());
    assertFloatNear(20.0f, buf.current()->delta_r_mm, 0.01f,
                    "push collinear merge: delta_r combined");
    assertFloatNear(20.0f, buf.current()->phys_distance_mm, 0.01f,
                    "push collinear merge: phys_distance combined");
}

void test_push_two_non_collinear() {
    // Two blocks at 90 degrees should NOT merge
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    buf.push(10.0f, 0.0f, 50.0f, 50.0f);   // pure radius
    buf.push(0.0f, 90.0f, 50.0f, 60.0f);   // pure theta

    TEST_ASSERT_EQUAL_UINT8(2, buf.count());

    // Second block's max_entry_speed should be < 50 (90-degree turn)
    const PlannerBlock& b1 = buf.block_at(1);
    TEST_ASSERT_TRUE(b1.max_entry_speed_mmps < 50.0f);
    TEST_ASSERT_TRUE(b1.max_entry_speed_mmps > 0.0f);
}

void test_push_dir_change_caps_speed() {
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    // First block: positive steps_r_total
    buf.push(10.0f, 0.0f, 50.0f, 50.0f);

    // Second block: large theta + small negative radius -> compensation dominates
    // steps_r_total will be negative if compensation > command
    // delta_r = +0.3mm -> 37.5 cmd steps, delta_theta = +2 deg -> 111 comp steps
    // radiusSteps = round(0.3*125 + 2*55.556) = round(37.5 + 111.11) = round(148.6) = 149
    // Hmm, compensation is ADDED not subtracted in the existing code.
    // So steps_r_total = cmd + comp = both positive here.
    // For a sign flip, we need delta_r negative:
    // delta_r = -2mm -> -250 cmd steps, delta_theta = +1 -> +55.6 comp
    // radiusSteps = round(-250 + 55.6) = round(-194.4) = -194 -> negative!
    buf.push(-2.0f, 1.0f, 50.0f, 50.0f);

    const PlannerBlock& b1 = buf.block_at(1);
    TEST_ASSERT_TRUE(b1.dir_change_warning);
    TEST_ASSERT_TRUE(b1.max_entry_speed_mmps <= DIR_CHANGE_MAX_SPEED_MMPS + 0.01f);
}

// ---------------------------------------------------------------------------
// Phase 2: Two-Pass Lookahead Tests (Cases G-I)
// ---------------------------------------------------------------------------

void test_case_G_collinear_merge() {
    // Three small collinear moves that fit within LUT limit should merge.
    // 5mm radius = 625 steps each. 3x625 = 1875 < AVR_MAX_PRECOMPUTE_STEPS (3000)
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    buf.push(5.0f, 0.0f, 50.0f, 50.0f);
    buf.push(5.0f, 0.0f, 50.0f, 55.0f);
    buf.push(5.0f, 0.0f, 50.0f, 60.0f);
    buf.mark_terminal();

    // All three should merge into a single 15mm block
    TEST_ASSERT_EQUAL_UINT8(1, buf.count());
    assertFloatNear(15.0f, buf.current()->delta_r_mm, 0.01f,
                    "G: merged delta_r = 15mm");
    assertFloatNear(0.0f, buf.current()->entry_speed_mmps, 0.01f,
                    "G: merged entry = 0 (start from rest)");
    assertFloatNear(0.0f, buf.current()->exit_speed_mmps, 0.01f,
                    "G: merged exit = 0 (terminal)");
}

void test_collinear_merge_lut_overflow_guard() {
    // Three 10mm radius moves = 3x1250 = 3750 steps > AVR_MAX_PRECOMPUTE_STEPS
    // First two merge (2500 < 3000), third cannot merge (2500+1250=3750 > 3000)
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    buf.push(10.0f, 0.0f, 50.0f, 50.0f);
    buf.push(10.0f, 0.0f, 50.0f, 60.0f);
    buf.push(10.0f, 0.0f, 50.0f, 70.0f);

    TEST_ASSERT_EQUAL_UINT8(2, buf.count());
    assertFloatNear(20.0f, buf.block_at(0).delta_r_mm, 0.01f,
                    "LUT guard: first two merged to 20mm");
    assertFloatNear(10.0f, buf.block_at(1).delta_r_mm, 0.01f,
                    "LUT guard: third block stays at 10mm");
}

void test_case_G_lookahead_non_collinear() {
    // Three non-collinear blocks to test actual two-pass lookahead
    // Block 0: (+10, 0)  radius only
    // Block 1: (+10, +5deg) slight mixed move (non-collinear)
    // Block 2: (+10, 0)  radius only
    // These should NOT merge (different directions)
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    buf.push(10.0f, 0.0f, 50.0f, 50.0f);
    buf.push(10.0f, 5.0f, 50.0f, 60.0f);
    buf.push(10.0f, 0.0f, 50.0f, 70.0f);
    buf.mark_terminal();

    TEST_ASSERT_EQUAL_UINT8(3, buf.count());

    // Block 0: entry = 0 (start from rest)
    const PlannerBlock& b0 = buf.block_at(0);
    assertFloatNear(0.0f, b0.entry_speed_mmps, 0.01f, "G2: b0 entry = 0");

    // Block 2: exit = 0 (terminal)
    const PlannerBlock& b2 = buf.block_at(2);
    assertFloatNear(0.0f, b2.exit_speed_mmps, 0.01f, "G2: b2 exit = 0");

    // All entries should be non-negative and <= nominal
    for (uint8_t i = 0; i < 3; i++) {
        const PlannerBlock& b = buf.block_at(i);
        TEST_ASSERT_TRUE(b.entry_speed_mmps >= 0.0f);
        TEST_ASSERT_TRUE(b.entry_speed_mmps <= 50.01f);
    }

    // Exit of b0 should equal entry of b1
    const PlannerBlock& b1 = buf.block_at(1);
    assertFloatNear(b1.entry_speed_mmps, b0.exit_speed_mmps, 0.01f,
                    "G2: b0 exit = b1 entry");
    assertFloatNear(b2.entry_speed_mmps, b1.exit_speed_mmps, 0.01f,
                    "G2: b1 exit = b2 entry");
}

void test_case_H_short_block() {
    // Block 1: (+100 mm, 0) at 50 mm/s
    // Block 2: (+0.5 mm, 0) at 50 mm/s (very short)
    // Block 3: (+100 mm, 0) at 50 mm/s
    // Backward pass should limit speeds through the short block.
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    buf.push(100.0f, 0.0f, 50.0f, 50.0f);
    buf.push(0.5f,   0.0f, 50.0f, 150.0f);
    buf.push(100.0f, 0.0f, 50.0f, 150.5f);
    buf.mark_terminal();

    TEST_ASSERT_EQUAL_UINT8(3, buf.count());

    // The short block (b1) should have limited entry/exit speeds
    const PlannerBlock& b1 = buf.block_at(1);

    // Block 2 needs to decel from b1.exit to 0 over 100mm — should be fine.
    // The bottleneck is block 1 (0.5mm): it needs to decel to b2.entry
    // and can only reach sqrt(2*80*0.5) = sqrt(80) = 8.94 mm/s from a standing start.
    // Backward from b2: b2 entry limited by sqrt(0 + 2*80*100) from terminal end
    // = sqrt(16000) = 126.5 mm/s -> capped at 50.

    // The short block should create a speed dip
    TEST_ASSERT_TRUE(b1.entry_speed_mmps <= 50.0f);

    // Verify exit speeds are consistent
    const PlannerBlock& b0 = buf.block_at(0);
    const PlannerBlock& b2 = buf.block_at(2);
    assertFloatNear(b1.entry_speed_mmps, b0.exit_speed_mmps, 0.01f,
                    "H: b0 exit = b1 entry");
    assertFloatNear(b2.entry_speed_mmps, b1.exit_speed_mmps, 0.01f,
                    "H: b1 exit = b2 entry");
    assertFloatNear(0.0f, b2.exit_speed_mmps, 0.01f, "H: b2 exit = 0");
}

void test_case_I_dir_change_cap() {
    // Block 1: positive steps_r_total
    // Block 2: steps_r_total flips sign (compensation dominates)
    // -> dir_change_warning set on block 2
    // -> max_entry_speed capped at DIR_CHANGE_MAX_SPEED_MMPS
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    buf.push(5.0f, 45.0f, 50.0f, 50.0f);   // positive r, positive theta
    buf.push(-2.0f, 1.0f, 50.0f, 55.0f);   // negative r_total (see earlier calc)
    buf.mark_terminal();

    TEST_ASSERT_EQUAL_UINT8(2, buf.count());

    const PlannerBlock& b1 = buf.block_at(1);
    TEST_ASSERT_TRUE(b1.dir_change_warning);
    TEST_ASSERT_TRUE(b1.max_entry_speed_mmps <= DIR_CHANGE_MAX_SPEED_MMPS + 0.01f);
    TEST_ASSERT_TRUE(b1.entry_speed_mmps <= DIR_CHANGE_MAX_SPEED_MMPS + 0.01f);
}

// ---------------------------------------------------------------------------
// Phase 2: Advance and clear
// ---------------------------------------------------------------------------

void test_advance() {
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    buf.push(10.0f, 0.0f, 50.0f, 50.0f);
    buf.push(20.0f, 0.0f, 50.0f, 60.0f);
    TEST_ASSERT_EQUAL_UINT8(2, buf.count());

    // Current should be the first block (10mm)
    assertFloatNear(10.0f, buf.current()->phys_distance_mm, 0.01f, "advance: first block");

    buf.advance();
    TEST_ASSERT_EQUAL_UINT8(1, buf.count());
    assertFloatNear(20.0f, buf.current()->phys_distance_mm, 0.01f, "advance: second block");

    buf.advance();
    TEST_ASSERT_EQUAL_UINT8(0, buf.count());
    TEST_ASSERT_TRUE(buf.empty());
}

void test_clear() {
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    buf.push(10.0f, 0.0f, 50.0f, 50.0f);
    buf.push(10.0f, 0.0f, 50.0f, 60.0f);
    buf.clear();

    TEST_ASSERT_EQUAL_UINT8(0, buf.count());
    TEST_ASSERT_TRUE(buf.empty());
}

// ---------------------------------------------------------------------------
// Phase 2: Mark terminal forces exit to 0
// ---------------------------------------------------------------------------

void test_mark_terminal() {
    PlannerBuffer buf;
    buf.init(DEFAULT_PHYS_ACCEL_MMPS2);

    // Use non-collinear blocks to prevent merge
    buf.push(10.0f, 0.0f, 50.0f, 50.0f);
    buf.push(0.0f, 90.0f, 50.0f, 60.0f);
    buf.mark_terminal();

    TEST_ASSERT_EQUAL_UINT8(2, buf.count());
    const PlannerBlock& last = buf.block_at(1);
    TEST_ASSERT_TRUE(last.is_terminal);
    assertFloatNear(0.0f, last.exit_speed_mmps, 0.01f, "terminal: exit = 0");
}

// ===== Test Runner =====

int main(int argc, char** argv) {
    UNITY_BEGIN();

    // Phase 1: Junction speed (Cases A-F)
    RUN_TEST(test_case_A_collinear_radius);
    RUN_TEST(test_case_B_90deg_turn_R50);
    RUN_TEST(test_case_C_90deg_turn_R5);
    RUN_TEST(test_case_D_reversal);
    RUN_TEST(test_case_E_mixed_move);
    RUN_TEST(test_case_F_below_R_min);
    RUN_TEST(test_sizeof_planner_block);

    // Phase 1: Push integration
    RUN_TEST(test_push_single_block);
    RUN_TEST(test_push_two_collinear_merge);
    RUN_TEST(test_push_two_non_collinear);
    RUN_TEST(test_push_dir_change_caps_speed);

    // Phase 2: Two-pass lookahead (Cases G-I)
    RUN_TEST(test_case_G_collinear_merge);
    RUN_TEST(test_collinear_merge_lut_overflow_guard);
    RUN_TEST(test_case_G_lookahead_non_collinear);
    RUN_TEST(test_case_H_short_block);
    RUN_TEST(test_case_I_dir_change_cap);

    // Phase 2: Buffer operations
    RUN_TEST(test_advance);
    RUN_TEST(test_clear);
    RUN_TEST(test_mark_terminal);

    return UNITY_END();
}
