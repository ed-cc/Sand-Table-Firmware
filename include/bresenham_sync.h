// bresenham_sync.h - Two-axis step distribution using Bresenham's line algorithm
//
// Distributes steps between a dominant axis (more steps) and a minor axis
// (fewer steps) so both axes complete their moves simultaneously. The dominant
// axis steps every tick; the minor axis steps when the error accumulator
// overflows.

#ifndef BRESENHAM_SYNC_H
#define BRESENHAM_SYNC_H

#include <stdint.h>

class BresenhamSync {
public:
    BresenhamSync();

    // Initialise for a new move. stepsA and stepsB are absolute (unsigned) step counts.
    // The axis with more steps becomes dominant.
    void init(int32_t stepsA, int32_t stepsB);

    // Call once per dominant-axis step. Returns true if the minor axis
    // should also step this tick.
    bool stepMinor();

    // True when all dominant steps have been consumed
    bool isComplete() const { return m_domRemaining <= 0; }

    // Steps remaining on the dominant axis
    int32_t dominantRemaining() const { return m_domRemaining; }

    // Which axis is dominant: true = A is dominant, false = B is dominant
    bool isADominant() const { return m_aDominant; }

    // Total steps for each axis (as initialised)
    int32_t totalDominant() const { return m_dominant; }
    int32_t totalMinor() const { return m_minor; }

private:
    int32_t m_dominant;
    int32_t m_minor;
    int32_t m_error;
    int32_t m_domRemaining;
    bool    m_aDominant;
};

#endif // BRESENHAM_SYNC_H
