// homing_manager.cpp - Endstop-based homing for TMC2208

#include "homing_manager.h"
#include "values.h"

#ifndef UNIT_TEST
#include <Arduino.h>
#endif

// Static endstop flags
volatile bool HomingManager::s_radiusEndstopTriggered = false;
volatile bool HomingManager::s_thetaEndstopTriggered = false;

HomingManager::HomingManager()
    : m_state(HOMING_IDLE)
{
}

void HomingManager::onRadiusEndstop() {
    s_radiusEndstopTriggered = true;
}

void HomingManager::onThetaEndstop() {
    s_thetaEndstopTriggered = true;
}

void HomingManager::setupEndstopInterrupts() {
#ifndef UNIT_TEST
    // Configure endstop pins with internal pullup (for NO switches to GND)
    pinMode(RADIUS_ENDSTOP_PIN, RADIUS_ENDSTOP_PULLUP ? INPUT_PULLUP : INPUT);
    pinMode(THETA_ENDSTOP_PIN, THETA_ENDSTOP_PULLUP ? INPUT_PULLUP : INPUT);

    // Attach interrupts — FALLING edge for NO switches with pullup
    attachInterrupt(digitalPinToInterrupt(RADIUS_ENDSTOP_PIN), onRadiusEndstop, FALLING);
    attachInterrupt(digitalPinToInterrupt(THETA_ENDSTOP_PIN), onThetaEndstop, FALLING);
#endif
}

void HomingManager::start(CoordinatedStepper& stepper) {
    setupEndstopInterrupts();

    s_radiusEndstopTriggered = false;
    s_thetaEndstopTriggered = false;

    // Start with radius homing: move inward (negative direction)
    m_state = HOMING_RADIUS_FAST_APPROACH;
    stepper.moveRelative(-RADIUS_MAX_HOMING_MM, 0.0f, HOMING_FAST_SPEED_MM_MIN);
}

bool HomingManager::run(CoordinatedStepper& stepper) {
    switch (m_state) {

    case HOMING_IDLE:
    case HOMING_COMPLETE:
        return false;

    case HOMING_RADIUS_FAST_APPROACH:
        if (s_radiusEndstopTriggered) {
            // Endstop hit — stop and back off
            s_radiusEndstopTriggered = false;
            // Force stop by setting position to current (cancels remaining steps)
            stepper.setPosition(stepper.getRadiusMM(), stepper.getThetaDegrees());
            // Back off
            stepper.moveRelative(HOMING_BACKOFF_MM, 0.0f, HOMING_FAST_SPEED_MM_MIN);
            m_state = HOMING_RADIUS_BACKOFF;
        } else if (stepper.isComplete()) {
            // Moved max distance without hitting endstop — error
            // For now, just stop (could report error)
            m_state = HOMING_IDLE;
            return false;
        }
        break;

    case HOMING_RADIUS_BACKOFF:
        if (stepper.isComplete()) {
            // Back-off done — slow approach
            s_radiusEndstopTriggered = false;
            stepper.moveRelative(-HOMING_BACKOFF_MM * 2, 0.0f, HOMING_SLOW_SPEED_MM_MIN);
            m_state = HOMING_RADIUS_SLOW_APPROACH;
        }
        break;

    case HOMING_RADIUS_SLOW_APPROACH:
        if (s_radiusEndstopTriggered) {
            // Second hit — this is our zero
            s_radiusEndstopTriggered = false;
            float currentTheta = stepper.getThetaDegrees();
            stepper.setPosition(0.0f, currentTheta);
            // Now home theta
            m_state = HOMING_THETA_FAST_APPROACH;
            stepper.moveRelative(0.0f, THETA_MAX_HOMING_DEGREES, HOMING_FAST_SPEED_MM_MIN);
        } else if (stepper.isComplete()) {
            m_state = HOMING_IDLE;
            return false;
        }
        break;

    case HOMING_THETA_FAST_APPROACH:
        if (s_thetaEndstopTriggered) {
            s_thetaEndstopTriggered = false;
            stepper.setPosition(stepper.getRadiusMM(), stepper.getThetaDegrees());
            stepper.moveRelative(0.0f, -HOMING_BACKOFF_DEG, HOMING_FAST_SPEED_MM_MIN);
            m_state = HOMING_THETA_BACKOFF;
        } else if (stepper.isComplete()) {
            m_state = HOMING_IDLE;
            return false;
        }
        break;

    case HOMING_THETA_BACKOFF:
        if (stepper.isComplete()) {
            s_thetaEndstopTriggered = false;
            stepper.moveRelative(0.0f, HOMING_BACKOFF_DEG * 2, HOMING_SLOW_SPEED_MM_MIN);
            m_state = HOMING_THETA_SLOW_APPROACH;
        }
        break;

    case HOMING_THETA_SLOW_APPROACH:
        if (s_thetaEndstopTriggered) {
            s_thetaEndstopTriggered = false;
            stepper.setPosition(0.0f, 0.0f);
            m_state = HOMING_COMPLETE;
            return false;
        } else if (stepper.isComplete()) {
            m_state = HOMING_IDLE;
            return false;
        }
        break;
    }

    return true;
}
