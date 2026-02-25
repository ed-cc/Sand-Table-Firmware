// motion_commander.cpp - Move command dispatcher
//
// Streaming dispatch: MOVE_TO/MOVE_RELATIVE are fed into the trajectory
// planner via queueMove() with backpressure (waits when planner buffer full).
// Non-motion commands (HOME, STOP, SET_POSITION, ENABLE, DISABLE) flush
// all queued motion first, then execute.

#include "motion_commander.h"
#include <math.h>

MotionCommander::MotionCommander(
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY>& moveBuffer,
    Motion& motion)
    : m_moveBuffer(moveBuffer)
    , m_motion(motion)
    , m_hasPending(false)
{}

bool MotionCommander::run() {
    // Grab the next command if we don't have one pending
    if (!m_hasPending) {
        if (m_moveBuffer.isEmpty()) {
            // No more commands — flush any buffered planner moves
            m_motion.endProgram();
            return false;
        }
        m_moveBuffer.dequeue(m_pending);
        m_hasPending = true;
    }

    // Try to dispatch the pending command
    if (tryDispatch(m_pending)) {
        m_hasPending = false;
        return true;
    }

    return false;
}

bool MotionCommander::tryDispatch(const MoveCommand& cmd) {
    switch (cmd.type) {
        case MOVE_TO: {
            if (!m_motion.isPositionKnown()) {
                Serial.println(F("error: Cannot move to absolute position before homing or G92"));
                return true;  // consumed (error), don't retry
            }
            if (m_motion.isPlannerFull()) return false;  // backpressure

            float r = isnan(cmd.radius) ? m_motion.getRadiusMM() : cmd.radius;
            float a = isnan(cmd.theta) ? m_motion.getThetaDegrees() : cmd.theta;
            m_motion.queueMove(r, a, true);
            return true;
        }

        case MOVE_RELATIVE: {
            if (m_motion.isPlannerFull()) return false;  // backpressure

            float dr = isnan(cmd.radius) ? 0.0f : cmd.radius;
            float da = isnan(cmd.theta) ? 0.0f : cmd.theta;
            m_motion.queueMove(dr, da, false);
            return true;
        }

        case MOVE_SET_SPEED:
            m_motion.setFeedrate(cmd.radius);
            return true;

        // Non-motion commands: flush planner first
        case MOVE_HOME:
            if (!m_motion.isMovementComplete()) return false;
            m_motion.home();
            return true;

        case MOVE_STOP:
            if (!m_motion.isMovementComplete()) return false;
            m_motion.disableMotors(true, true);
            return true;

        case MOVE_SET_POSITION: {
            if (!m_motion.isMovementComplete()) return false;
            if (!m_motion.isPositionKnown() && (isnan(cmd.radius) || isnan(cmd.theta))) {
                Serial.println(F("error: G92 requires both R and A before position is known"));
                return true;
            }
            float r = isnan(cmd.radius) ? m_motion.getRadiusMM() : cmd.radius;
            float a = isnan(cmd.theta) ? m_motion.getThetaDegrees() : cmd.theta;
            m_motion.setPosition(r, a);
            return true;
        }

        case MOVE_ENABLE:
            if (!m_motion.isMovementComplete()) return false;
            m_motion.enableMotors(cmd.radius > 0, cmd.theta > 0);
            return true;

        case MOVE_DISABLE:
            if (!m_motion.isMovementComplete()) return false;
            m_motion.disableMotors(cmd.radius > 0, cmd.theta > 0);
            return true;

        case MOVE_NONE:
            return true;
    }
    return true;
}
