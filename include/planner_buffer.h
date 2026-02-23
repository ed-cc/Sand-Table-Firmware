// planner_buffer.h - Lookahead planner block buffer with junction speed blending
//
// Stores a ring buffer of PlannerBlock structs representing queued moves.
// Each block carries both planning-domain fields (physical mm/s) and
// execution-domain fields (motor steps). The two-pass lookahead optimiser
// (_replan) computes entry/exit speeds for seamless junction blending.
//
// Planning domain: delta_r_mm, delta_theta_deg, R_start_mm, phys_distance_mm,
//   unit_phys[], nominal/entry/exit/max_entry speeds. All in physical mm/s.
//   Compensation steps NEVER enter this domain.
//
// Execution domain: steps_theta, steps_r_total (includes compensation).
//   Fed to BlockMeta and Bresenham for ISR consumption.

#ifndef PLANNER_BUFFER_H
#define PLANNER_BUFFER_H

#include <stdint.h>
#include "values.h"

struct PlannerBlock {
    // -- Set at parse time (planning domain) --
    float  delta_r_mm;           // commanded radius change [mm]
    float  delta_theta_deg;      // commanded theta change [degrees]
    float  nominal_speed_mmps;   // feed rate [mm/s physical distance]
    float  R_start_mm;           // radius position at block start [mm]

    // Derived commanded-motion distance (NO compensation steps)
    float  phys_distance_mm;     // sqrt(delta_r_mm^2 + arc_mm^2)
    float  unit_phys[2];         // {delta_r_mm, arc_mm} / phys_distance_mm

    // Step counts (execution domain, from ThetaCompensation::decompose())
    int32_t steps_theta;
    int32_t steps_r_total;       // commanded + compensation

    // -- Set by lookahead pass --
    float  entry_speed_mmps;     // v at block start [mm/s]
    float  exit_speed_mmps;      // v at block end   [mm/s]
    float  max_entry_speed_mmps; // junction speed limit from geometry

    // -- Flags --
    bool   recalculate;
    bool   is_terminal;          // last block before a required stop
    bool   dir_change_warning;   // radius DIR changes at entry to this block
};

// Compute the maximum junction entry speed between two consecutive blocks
// using the physical-space junction deviation formula.
// Returns speed in mm/s, capped to the minimum of both blocks' nominal speeds.
float compute_max_entry_speed(const PlannerBlock& prev,
                              const PlannerBlock& curr,
                              float junction_deviation_mm,
                              float accel_mmps2);

class PlannerBuffer {
public:
    PlannerBuffer();

    void  init(float accel_mmps2);

    // Push a new block. Calls ThetaCompensation::decompose(), computes
    // physical vectors, junction speed, attempts merge, and runs _replan().
    void  push(float delta_r_mm, float delta_theta_deg,
               float feed_mmps, float R_current_mm);

    // Mark the newest block as terminal (exit speed = 0).
    void  mark_terminal();

    // Discard all blocks (abort path).
    void  clear();

    bool     full() const;
    bool     empty() const;
    uint8_t  count() const;

    // Called by segment preparation when a block is fully consumed.
    void  advance();

    // Block currently feeding into segment prep (oldest unconsumed).
    PlannerBlock*       current();
    const PlannerBlock* current() const;

    // Next block after current (for DIR pre-loading).
    PlannerBlock*       peek_next();
    const PlannerBlock* peek_next() const;

    // Direct indexed access (for testing). Index 0 = tail (oldest).
    PlannerBlock&       block_at(uint8_t logical_index);
    const PlannerBlock& block_at(uint8_t logical_index) const;

private:
    PlannerBlock _buf[PLANNER_BUFFER_CAPACITY];
    uint8_t      _head;   // next write position
    uint8_t      _tail;   // oldest unconsumed block
    uint8_t      _count;
    float        _accel;  // physical acceleration [mm/s^2]

    void  _replan();
    void  _compute_physical_vector(PlannerBlock& blk);
    bool  _try_merge(PlannerBlock& prev, const PlannerBlock& curr);
};

#endif // PLANNER_BUFFER_H
