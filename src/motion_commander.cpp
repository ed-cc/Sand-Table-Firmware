// motion_commander.cpp - Move command dispatcher
#include "motion_commander.h"
#include <math.h>

MotionCommander::MotionCommander(
    RingBuffer<MoveCommand, MOVE_BUFFER_CAPACITY>& moveBuffer,
    Motion& motion)
    : m_moveBuffer(moveBuffer)
    , m_motion(motion)
{}

bool MotionCommander::run() {
    if (m_moveBuffer.isEmpty()) return false;
    if (!m_motion.isMovementComplete()) return false;

    MoveCommand cmd;
    m_moveBuffer.dequeue(cmd);
    dispatch(cmd);
    return true;
}

void MotionCommander::dispatch(const MoveCommand& cmd) {
    switch (cmd.type) {
        case MOVE_TO: {
            if (!m_motion.isPositionKnown()) {
                Serial.println(F("error: Cannot move to absolute position before homing or G92"));
                break;
            }
            float r = isnan(cmd.radius) ? m_motion.getRadiusMM() : cmd.radius;
            float a = isnan(cmd.theta) ? m_motion.getThetaDegrees() : cmd.theta;
            Serial.print(F("[CMD] MOVE_TO R="));
            Serial.print(r, 2);
            Serial.print(F(" A="));
            Serial.print(a, 2);
            Serial.print(F(" F="));
            Serial.println(m_motion.getFeedrate(), 1);
            m_motion.moveTo(r, a);
            break;
        }
        case MOVE_RELATIVE:
            Serial.print(F("[CMD] MOVE_REL dR="));
            Serial.print(cmd.radius, 2);
            Serial.print(F(" dA="));
            Serial.println(cmd.theta, 2);
            m_motion.moveRelative(cmd.radius, cmd.theta);
            break;

        case MOVE_HOME:
            Serial.println(F("[CMD] HOME"));
            m_motion.home();
            break;

        case MOVE_STOP:
            Serial.println(F("[CMD] STOP"));
            m_motion.disableMotors(true, true);
            break;

        case MOVE_SET_POSITION: {
            if (!m_motion.isPositionKnown() && (isnan(cmd.radius) || isnan(cmd.theta))) {
                Serial.println(F("error: G92 requires both R and A before position is known"));
                break;
            }
            float r = isnan(cmd.radius) ? m_motion.getRadiusMM() : cmd.radius;
            float a = isnan(cmd.theta) ? m_motion.getThetaDegrees() : cmd.theta;
            Serial.print(F("[CMD] SET_POS R="));
            Serial.print(r, 2);
            Serial.print(F(" A="));
            Serial.println(a, 2);
            m_motion.setPosition(r, a);
            break;
        }

        case MOVE_SET_SPEED:
            Serial.print(F("[CMD] SET_SPEED F="));
            Serial.println(cmd.radius, 1);
            m_motion.setFeedrate(cmd.radius);
            break;

        case MOVE_NONE:
            break;

        case MOVE_ENABLE:
            Serial.println(F("[CMD] ENABLE"));
            m_motion.enableMotors(cmd.radius > 0, cmd.theta > 0);
            break;

        case MOVE_DISABLE:
            Serial.println(F("[CMD] DISABLE"));
            m_motion.disableMotors(cmd.radius > 0, cmd.theta > 0);
            break;
    }
}
