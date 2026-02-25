// planner_buffer.cpp - Lookahead planner block buffer implementation

#include "planner_buffer.h"
#include "theta_compensation.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ---------------------------------------------------------------------------
// Junction speed calculation (physical mm-space)
// ---------------------------------------------------------------------------

float compute_max_entry_speed(const PlannerBlock& prev,
                              const PlannerBlock& curr,
                              float junction_deviation_mm,
                              float accel_mmps2) {
    // Either block is a dwell (zero distance) — require full stop
    if (prev.phys_distance_mm < 0.001f || curr.phys_distance_mm < 0.001f) {
        return 0.0f;
    }

    // Dot product of physical unit vectors (negated per junction deviation formula)
    float cos_theta = -(prev.unit_phys[0] * curr.unit_phys[0]
                      + prev.unit_phys[1] * curr.unit_phys[1]);

    // Collinear (same direction): no slowdown needed
    if (cos_theta <= -0.9999f) {
        float v_min = prev.nominal_speed_mmps;
        if (curr.nominal_speed_mmps < v_min) v_min = curr.nominal_speed_mmps;
        return v_min;
    }

    // Reversal (opposite direction): must stop
    if (cos_theta >= 0.9999f) {
        return 0.0f;
    }

    float sin_d2 = sqrtf(0.5f * (1.0f - cos_theta));
    float denom  = 1.0f - sin_d2;

    // Degenerate (very sharp corner): force stop
    if (denom < 1e-5f) return 0.0f;

    float R_jd = junction_deviation_mm * sin_d2 / denom;
    float v_max_sqr = R_jd * accel_mmps2;

    float v_limit = sqrtf(v_max_sqr);

    // Cap to the lower of the two blocks' nominal speeds
    float v_cap = prev.nominal_speed_mmps;
    if (curr.nominal_speed_mmps < v_cap) v_cap = curr.nominal_speed_mmps;
    if (v_limit > v_cap) v_limit = v_cap;

    return v_limit;
}

// ---------------------------------------------------------------------------
// PlannerBuffer
// ---------------------------------------------------------------------------

PlannerBuffer::PlannerBuffer()
    : _head(0), _tail(0), _count(0), _accel(DEFAULT_PHYS_ACCEL_MMPS2)
{
    memset(_buf, 0, sizeof(_buf));
}

void PlannerBuffer::init(float accel_mmps2) {
    _accel = accel_mmps2;
    clear();
}

void PlannerBuffer::clear() {
    _head = 0;
    _tail = 0;
    _count = 0;
}

bool PlannerBuffer::full() const {
    return _count >= PLANNER_BUFFER_CAPACITY;
}

bool PlannerBuffer::empty() const {
    return _count == 0;
}

uint8_t PlannerBuffer::count() const {
    return _count;
}

void PlannerBuffer::advance() {
    if (_count == 0) return;
    _tail = (_tail + 1) % PLANNER_BUFFER_CAPACITY;
    _count--;
}

PlannerBlock* PlannerBuffer::current() {
    if (_count == 0) return nullptr;
    return &_buf[_tail];
}

const PlannerBlock* PlannerBuffer::current() const {
    if (_count == 0) return nullptr;
    return &_buf[_tail];
}

PlannerBlock* PlannerBuffer::peek_next() {
    if (_count < 2) return nullptr;
    return &_buf[(_tail + 1) % PLANNER_BUFFER_CAPACITY];
}

const PlannerBlock* PlannerBuffer::peek_next() const {
    if (_count < 2) return nullptr;
    return &_buf[(_tail + 1) % PLANNER_BUFFER_CAPACITY];
}

PlannerBlock& PlannerBuffer::block_at(uint8_t logical_index) {
    return _buf[(_tail + logical_index) % PLANNER_BUFFER_CAPACITY];
}

const PlannerBlock& PlannerBuffer::block_at(uint8_t logical_index) const {
    return _buf[(_tail + logical_index) % PLANNER_BUFFER_CAPACITY];
}

// ---------------------------------------------------------------------------
// Physical vector computation
// ---------------------------------------------------------------------------

void PlannerBuffer::_compute_physical_vector(PlannerBlock& blk) {
    float arc_mm = 0.0f;

    float absR = blk.R_start_mm;
    if (absR < 0.0f) absR = -absR;

    if (absR >= PLANNER_R_MIN_MM) {
        float delta_theta_rad = blk.delta_theta_deg * (float)M_PI / 180.0f;
        arc_mm = absR * delta_theta_rad;
    }
    // else: R below minimum, arc treated as zero (force stop at junctions)

    float dr2 = blk.delta_r_mm * blk.delta_r_mm;
    float arc2 = arc_mm * arc_mm;
    blk.phys_distance_mm = sqrtf(dr2 + arc2);

    if (blk.phys_distance_mm < 0.001f) {
        // Dwell / zero-distance block
        blk.unit_phys[0] = 0.0f;
        blk.unit_phys[1] = 0.0f;
    } else {
        blk.unit_phys[0] = blk.delta_r_mm / blk.phys_distance_mm;
        blk.unit_phys[1] = arc_mm / blk.phys_distance_mm;
    }
}

// ---------------------------------------------------------------------------
// Collinear merge
// ---------------------------------------------------------------------------

bool PlannerBuffer::_try_merge(PlannerBlock& prev, const PlannerBlock& curr) {
    // Guard 1: same commanded direction (dot product of physical unit vectors)
    float dot = prev.unit_phys[0] * curr.unit_phys[0]
              + prev.unit_phys[1] * curr.unit_phys[1];
    if (dot < COLLINEAR_DOT_THRESHOLD) return false;

    // Guard 2: same feed rate (within 1%)
    float diff = prev.nominal_speed_mmps - curr.nominal_speed_mmps;
    if (diff < 0) diff = -diff;
    if (diff > 0.01f * prev.nominal_speed_mmps) return false;

    // Guard 3: merged step count does not overflow the LUT
    int32_t abs_theta = prev.steps_theta + curr.steps_theta;
    if (abs_theta < 0) abs_theta = -abs_theta;
    int32_t abs_r = prev.steps_r_total + curr.steps_r_total;
    if (abs_r < 0) abs_r = -abs_r;
    int32_t merged_dom = abs_theta > abs_r ? abs_theta : abs_r;
    if (merged_dom > AVR_MAX_PRECOMPUTE_STEPS) return false;

    // Guard 4: no direction change on radius axis across the merge
    if (prev.dir_change_warning || curr.dir_change_warning) return false;

    // Guard 5: neither block is terminal
    if (prev.is_terminal) return false;

    // All guards passed — merge curr into prev
    prev.delta_r_mm      += curr.delta_r_mm;
    prev.delta_theta_deg += curr.delta_theta_deg;
    prev.steps_theta     += curr.steps_theta;
    prev.steps_r_total   += curr.steps_r_total;

    // Recompute physical distance with updated deltas (use prev.R_start_mm)
    _compute_physical_vector(prev);

    return true;
}

// ---------------------------------------------------------------------------
// Two-pass lookahead (stub in Phase 1, implemented in Phase 2)
// ---------------------------------------------------------------------------

void PlannerBuffer::_replan() {
    if (_count < 2) return;

    // --- Forward pass ---
    // First block entry speed is already set (0 for first move, or committed).
    for (uint8_t i = 1; i < _count; i++) {
        PlannerBlock& prev = _buf[(_tail + i - 1) % PLANNER_BUFFER_CAPACITY];
        PlannerBlock& curr = _buf[(_tail + i)     % PLANNER_BUFFER_CAPACITY];

        float v_reachable_sqr = prev.entry_speed_mmps * prev.entry_speed_mmps
                              + 2.0f * _accel * prev.phys_distance_mm;
        float v_reachable = sqrtf(v_reachable_sqr);

        float v_new = curr.max_entry_speed_mmps;
        if (v_reachable < v_new) v_new = v_reachable;
        if (curr.nominal_speed_mmps < v_new) v_new = curr.nominal_speed_mmps;
        curr.entry_speed_mmps = v_new;
        curr.recalculate = true;
    }

    // --- Backward pass ---
    // The last block's exit speed is 0 if terminal.
    PlannerBlock& last = _buf[(_tail + _count - 1) % PLANNER_BUFFER_CAPACITY];
    last.exit_speed_mmps = last.is_terminal ? 0.0f : last.exit_speed_mmps;

    for (int8_t i = (int8_t)_count - 2; i >= 0; i--) {
        PlannerBlock& curr = _buf[(_tail + i)     % PLANNER_BUFFER_CAPACITY];
        PlannerBlock& next = _buf[(_tail + i + 1) % PLANNER_BUFFER_CAPACITY];

        // What entry speed can next achieve given deceleration across curr?
        float v_limited_sqr = next.entry_speed_mmps * next.entry_speed_mmps
                            + 2.0f * _accel * curr.phys_distance_mm;
        float v_limited = sqrtf(v_limited_sqr);

        if (v_limited < curr.entry_speed_mmps) {
            curr.entry_speed_mmps = v_limited;
            curr.recalculate = true;
        }
    }

    // --- Propagate exit speeds ---
    for (uint8_t i = 0; i < _count - 1; i++) {
        PlannerBlock& curr = _buf[(_tail + i)     % PLANNER_BUFFER_CAPACITY];
        PlannerBlock& next = _buf[(_tail + i + 1) % PLANNER_BUFFER_CAPACITY];
        curr.exit_speed_mmps = next.entry_speed_mmps;
    }

    // Terminal block always exits at 0
    _buf[(_tail + _count - 1) % PLANNER_BUFFER_CAPACITY].exit_speed_mmps = 0.0f;
}

// ---------------------------------------------------------------------------
// Push
// ---------------------------------------------------------------------------

void PlannerBuffer::push(float delta_r_mm, float delta_theta_deg,
                         float feed_mmps, float R_current_mm) {
    if (full()) return;

    // Build the new block
    PlannerBlock blk;
    memset(&blk, 0, sizeof(blk));

    blk.delta_r_mm         = delta_r_mm;
    blk.delta_theta_deg    = delta_theta_deg;
    blk.nominal_speed_mmps = feed_mmps;
    blk.R_start_mm         = R_current_mm;

    // Execution domain: decompose into motor steps
    MoveSteps ms = ThetaCompensation::decompose(delta_r_mm, delta_theta_deg);
    blk.steps_theta   = ms.thetaSteps;
    blk.steps_r_total = ms.radiusSteps;

    // Planning domain: compute physical vector
    _compute_physical_vector(blk);

    // Detect direction change on radius axis vs. previous block
    blk.dir_change_warning = false;
    if (_count > 0) {
        PlannerBlock& prev = _buf[(_head - 1 + PLANNER_BUFFER_CAPACITY) % PLANNER_BUFFER_CAPACITY];
        bool prev_dir_positive = (prev.steps_r_total >= 0);
        bool curr_dir_positive = (blk.steps_r_total >= 0);
        blk.dir_change_warning = (prev_dir_positive != curr_dir_positive);
    }

    // Compute max entry speed from junction geometry
    if (_count == 0) {
        // First block: start from rest
        blk.max_entry_speed_mmps = 0.0f;
        blk.entry_speed_mmps = 0.0f;
    } else {
        PlannerBlock& prev = _buf[(_head - 1 + PLANNER_BUFFER_CAPACITY) % PLANNER_BUFFER_CAPACITY];
        blk.max_entry_speed_mmps = compute_max_entry_speed(
            prev, blk, JUNCTION_DEVIATION_MM, _accel);

        // R below minimum: force junction speed to zero
        if (blk.R_start_mm < PLANNER_R_MIN_MM) {
            blk.max_entry_speed_mmps = 0.0f;
        }

        // Direction change: cap to safe low speed
        if (blk.dir_change_warning) {
            if (blk.max_entry_speed_mmps > DIR_CHANGE_MAX_SPEED_MMPS) {
                blk.max_entry_speed_mmps = DIR_CHANGE_MAX_SPEED_MMPS;
            }
        }

        blk.entry_speed_mmps = blk.max_entry_speed_mmps;
    }

    blk.exit_speed_mmps = 0.0f;
    blk.is_terminal = false;
    blk.recalculate = true;

    // Attempt collinear merge with previous block
    if (_count > 0) {
        PlannerBlock& prev = _buf[(_head - 1 + PLANNER_BUFFER_CAPACITY) % PLANNER_BUFFER_CAPACITY];
        if (_try_merge(prev, blk)) {
            // Merged into prev — do not insert a new block
            _replan();
            return;
        }
    }

    // Insert new block
    _buf[_head] = blk;
    _head = (_head + 1) % PLANNER_BUFFER_CAPACITY;
    _count++;

    _replan();
}

// ---------------------------------------------------------------------------
// Mark terminal
// ---------------------------------------------------------------------------

void PlannerBuffer::mark_terminal() {
    if (_count == 0) return;
    uint8_t newest = (_head - 1 + PLANNER_BUFFER_CAPACITY) % PLANNER_BUFFER_CAPACITY;
    _buf[newest].is_terminal = true;
    _buf[newest].exit_speed_mmps = 0.0f;
    _replan();
}
