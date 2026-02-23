// bresenham_sync.cpp - Two-axis step distribution using Bresenham's line algorithm

#include "bresenham_sync.h"

BresenhamSync::BresenhamSync()
    : m_dominant(0)
    , m_minor(0)
    , m_error(0)
    , m_domRemaining(0)
    , m_aDominant(true)
{
}

void BresenhamSync::init(int32_t stepsA, int32_t stepsB) {
    int32_t absA = (stepsA >= 0) ? stepsA : -stepsA;
    int32_t absB = (stepsB >= 0) ? stepsB : -stepsB;

    if (absA >= absB) {
        m_dominant = absA;
        m_minor = absB;
        m_aDominant = true;
    } else {
        m_dominant = absB;
        m_minor = absA;
        m_aDominant = false;
    }

    m_error = m_dominant / 2;
    m_domRemaining = m_dominant;
}

bool BresenhamSync::stepMinor() {
    if (m_domRemaining <= 0) return false;

    m_domRemaining--;

    m_error += m_minor;
    if (m_error >= m_dominant) {
        m_error -= m_dominant;
        return true;
    }
    return false;
}
