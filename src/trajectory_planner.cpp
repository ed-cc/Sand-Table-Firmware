// trajectory_planner.cpp - Multi-block trajectory planner implementation
//
// Bridges planner buffer (junction speed planning) with timer backend ISR
// (step execution). Prepares BlockMeta + LUT from planned blocks, then
// hands off to the hardware for seamless multi-block execution.
//
// Streaming model:
//   - queue_move() auto-starts execution when buffer fills (8 blocks).
//   - run() auto-starts the next batch when ISR completes and ≥2 blocks
//     are queued (need ≥2 for junction speed planning).
//   - end_of_program() flushes any remaining blocks regardless of count.

#ifndef UNIT_TEST

#include "trajectory_planner.h"
#include "theta_compensation.h"
#include "timer_backend.h"
#include <Arduino.h>
#include <math.h>

TrajectoryPlanner::TrajectoryPlanner()
    : m_thetaAxis(nullptr)
    , m_radiusAxis(nullptr)
    , m_executing(false)
    , m_feedHoldActive(false)
    , m_splitActive(false)
{
}

void TrajectoryPlanner::init(StepperAxis* thetaAxis, StepperAxis* radiusAxis) {
    m_thetaAxis = thetaAxis;
    m_radiusAxis = radiusAxis;
    m_buffer.init(DEFAULT_PHYS_ACCEL_MMPS2);
}

void TrajectoryPlanner::queue_move(float delta_r_mm, float delta_theta_deg,
                                    float feed_mmps, float R_current_mm) {
    m_buffer.push(delta_r_mm, delta_theta_deg, feed_mmps, R_current_mm);

    // Auto-start execution when buffer is full
    if (!m_executing && m_buffer.full()) {
        _start_execution();
    }
}

void TrajectoryPlanner::end_of_program() {
    if (m_buffer.empty()) return;
    m_buffer.mark_terminal();
    if (!m_executing) {
        _start_execution();
    }
}

void TrajectoryPlanner::synchronise() {
    if (!m_executing && !m_buffer.empty()) {
        end_of_program();
    }

    // Poll until execution completes (including split batches)
    while (m_executing || m_splitActive) {
        run();
    }
}

bool TrajectoryPlanner::is_moving() const {
    if (m_splitActive) return true;
    return m_executing && !timerBackendIsComplete();
}

bool TrajectoryPlanner::buffer_full() const {
    return m_buffer.full();
}

bool TrajectoryPlanner::has_queued_blocks() const {
    return !m_buffer.empty();
}

bool TrajectoryPlanner::is_idle() const {
    return !m_executing && !m_splitActive && m_buffer.empty();
}

void TrajectoryPlanner::feed_hold() {
    if (!m_executing) return;
    m_feedHoldActive = true;
    timerBackendAbort();  // ISR will forced-decel to stop
}

void TrajectoryPlanner::resume() {
    if (!m_feedHoldActive) return;
    m_feedHoldActive = false;
    if (!m_buffer.empty()) {
        _start_execution();
    }
}

void TrajectoryPlanner::abort() {
    timerBackendAbort();  // ISR forced-decel

    // Wait for ISR to actually stop
    while (!timerBackendIsComplete() && !timerBackendIsStalled()) {
        // spin
    }

    m_buffer.clear();
    m_executing = false;
    m_feedHoldActive = false;
    m_splitActive = false;
}

void TrajectoryPlanner::run() {
    if (m_executing) {
        if (timerBackendIsComplete() || timerBackendIsStalled()) {
            m_executing = false;

            if (timerBackendIsStalled()) {
                Serial.println(F("[TP] Stalled"));
            }
        }
    }

    // Split continuation: feed next batch before normal auto-start
    if (!m_executing && m_splitActive) {
        if (m_splitDomFired >= m_splitTotalDom) {
            // All batches done — consume the oversized block
            m_splitActive = false;
            m_buffer.advance();
            // Fall through to normal auto-start for remaining blocks
        } else {
            _start_split_batch();
            return;
        }
    }

    // Auto-start next batch when ISR is idle.
    // Need ≥2 blocks for meaningful junction speed planning.
    // A single terminal block also starts immediately (exit_speed already 0).
    if (!m_executing && !m_buffer.empty()) {
        if (m_buffer.count() >= 2 || m_buffer.current()->is_terminal) {
            _start_execution();
        }
    }
}

void TrajectoryPlanner::_start_split_batch() {
    int32_t remaining = m_splitTotalDom - m_splitDomFired;
    int32_t batchDom = (remaining > AVR_MAX_PRECOMPUTE_STEPS)
                        ? AVR_MAX_PRECOMPUTE_STEPS : remaining;

    // Compute raw per-step intervals for this batch slice
    uint16_t* lut = timerBackendGetLutBuffer();
    int32_t entries = m_splitProfile.computeIntervalBlock(
        m_splitDomFired, lut, batchDom, F_CPU);

    // Proportional minor steps (cumulative tracking for exact total)
    int32_t cumDomAfter = m_splitDomFired + batchDom;
    int32_t cumMinorAfter = (int32_t)(
        (float)cumDomAfter * m_splitTotalMinor / m_splitTotalDom + 0.5f);
    int32_t batchMinor = cumMinorAfter - m_splitMinorFired;

    // Build single BlockMeta
    BlockMeta meta;
    meta.lut_start      = 0;
    meta.lut_count       = (uint16_t)entries;
    meta.dir_theta       = m_splitDirTheta;
    meta.dir_radius      = m_splitDirRadius;
    meta.bres_dom        = batchDom;
    meta.bres_minor      = batchMinor;
    meta.theta_dominant  = m_splitThetaDom;

    // Set directions
    m_thetaAxis->setDirection(meta.dir_theta);
    m_radiusAxis->setDirection(meta.dir_radius);

    timerBackendLoadPlan(&meta, 1, (uint16_t)entries,
                         m_thetaAxis, m_radiusAxis);
    m_executing = true;

    m_splitDomFired   += batchDom;
    m_splitMinorFired += batchMinor;
}

void TrajectoryPlanner::_start_execution() {
    if (m_buffer.empty() || m_thetaAxis == nullptr || m_radiusAxis == nullptr) return;

    uint8_t blockCount = m_buffer.count();

    // Temporary array for BlockMeta (on stack, ~128 bytes)
    BlockMeta metas[PLANNER_BUFFER_CAPACITY];
    // Write intervals directly into the ISR's LUT buffer (no copy needed)
    uint16_t* lut = timerBackendGetLutBuffer();
    uint16_t lutCursor = 0;

    for (uint8_t i = 0; i < blockCount; i++) {
        const PlannerBlock& blk = m_buffer.block_at(i);

        // Determine dominant axis and step counts
        int32_t absTheta = (blk.steps_theta >= 0) ? blk.steps_theta : -blk.steps_theta;
        int32_t absRadius = (blk.steps_r_total >= 0) ? blk.steps_r_total : -blk.steps_r_total;
        bool thetaDom = (absTheta >= absRadius);
        int32_t domSteps = thetaDom ? absTheta : absRadius;
        int32_t minSteps = thetaDom ? absRadius : absTheta;

        if (domSteps <= 0) continue;  // skip zero-length blocks

        if (blk.phys_distance_mm < 0.001f) {
            // Negligible ball displacement but motors need to move
            // (e.g., pure theta rotation near table center with compensation).
            // Plan directly in step-space using the dominant axis's speed limits.
            float maxVel, maxAccel, jerk;
            if (thetaDom) {
                maxVel   = (float)THETA_MAX_SPEED;
                maxAccel = (float)THETA_ACCELERATION;
                jerk     = THETA_JERK;
            } else {
                maxVel   = (float)RADIUS_MAX_SPEED;
                maxAccel = (float)RADIUS_ACCELERATION;
                jerk     = RADIUS_JERK;
            }
            m_profile.plan(domSteps, maxVel, maxAccel, jerk, 0.0f, 0.0f);
        } else {
            // Compute steps_per_mm for the dominant axis
            float steps_per_mm = (float)domSteps / blk.phys_distance_mm;

            // Plan the S-curve profile for this block
            m_profile.planPhysical(
                blk.phys_distance_mm,
                blk.entry_speed_mmps,
                blk.nominal_speed_mmps,
                blk.exit_speed_mmps,
                DEFAULT_PHYS_ACCEL_MMPS2,
                DEFAULT_PHYS_JERK_MMPS3,
                steps_per_mm
            );
        }

        // Check LUT space (using compressed format for multi-block)
        int32_t profileSteps = m_profile.lutEntryCount();
        int32_t available = AVR_MAX_PRECOMPUTE_STEPS - lutCursor;
        if (profileSteps > available) {
            if (i == 0) {
                // First block overflows: enter batch-split mode.
                // Plan the full profile once, then feed to ISR in batches.
                m_splitProfile    = m_profile;
                m_splitTotalDom   = domSteps;
                m_splitDomFired   = 0;
                m_splitTotalMinor = minSteps;
                m_splitMinorFired = 0;
                m_splitDirTheta   = (blk.steps_theta >= 0) ? 1 : -1;
                m_splitDirRadius  = (blk.steps_r_total >= 0) ? 1 : -1;
                m_splitThetaDom   = thetaDom;
                m_splitActive     = true;

                _start_split_batch();
                return;  // don't advance buffer yet
            }
            // Block K>0 overflows: truncate to K blocks, execute those.
            // The oversized block becomes block 0 on the next call.
            blockCount = i;
            break;
        }

        // Compute compressed intervals into LUT
        int32_t entries = m_profile.precomputeCompressedLUT(&lut[lutCursor], available, F_CPU);

        // Fill BlockMeta
        metas[i].lut_start = lutCursor;
        metas[i].lut_count = (uint16_t)entries;
        metas[i].dir_theta = (blk.steps_theta >= 0) ? 1 : -1;
        metas[i].dir_radius = (blk.steps_r_total >= 0) ? 1 : -1;
        metas[i].bres_dom = domSteps;
        metas[i].bres_minor = minSteps;
        metas[i].theta_dominant = thetaDom;

        lutCursor += entries;
    }

    if (blockCount == 0 || lutCursor == 0) {
        m_buffer.clear();
        return;
    }

    // Set directions for first block
    m_thetaAxis->setDirection(metas[0].dir_theta);
    m_radiusAxis->setDirection(metas[0].dir_radius);

    Serial.print(F("[TP] Exec: "));
    Serial.print(blockCount);
    Serial.print(F("blk "));
    Serial.print(lutCursor);
    Serial.println(F("lut"));

    // Hand off to hardware
    timerBackendLoadPlan(metas, blockCount, lutCursor,
                         m_thetaAxis, m_radiusAxis);
    m_executing = true;

    // Advance buffer past all consumed blocks
    for (uint8_t i = 0; i < blockCount; i++) {
        m_buffer.advance();
    }
}

#endif // UNIT_TEST
