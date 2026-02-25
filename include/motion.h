// motion.h - Motion control wrapper for Sand Table
//
// Thin wrapper around CoordinatedStepper providing:
// - TMC2208 driver setup and configuration
// - Homing sequence (endstop-based)
// - Motor enable/disable
// - Feedrate management
// - Position safety limits
//
// All motion planning, stepping, compensation, and unit conversion
// are handled by CoordinatedStepper.

#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "values.h"
#include "coordinated_stepper.h"
#include "homing_manager.h"
#include "trajectory_planner.h"

class Motion
{
public:
    Motion();

    // Initialize the motion system (TMC drivers and steppers)
    void setupSteppers();

    // Find home position using endstops
    void home();

    // Move to a specific position in user units (mm, degrees)
    void moveTo(float radiusMM, float thetaDegrees);

    // Move relative to current position (mm, degrees)
    void moveRelative(float radiusMM, float thetaDegrees);

    // Non-blocking run - call this repeatedly in loop()
    void run();

    // Check if both motors have reached their target positions
    bool isMovementComplete();

    // Check if homing is in progress
    bool isHoming() { return m_isHoming; }

    // Enable/disable motors
    void enableMotors(bool radius = true, bool theta = true);
    void disableMotors(bool radius = true, bool theta = true);

    // Get current position in user units
    float getRadiusMM();
    float getThetaDegrees();

    // Set current position without moving (G92)
    void setPosition(float radiusMM, float thetaDegrees);

    // Whether we have a known position (after homing or G92)
    bool isPositionKnown() const { return m_positionKnown; }

    // Feedrate (mm/min)
    void setFeedrate(float feedrate) { m_feedrate = feedrate; }
    float getFeedrate() const { return m_feedrate; }

    // Lookahead planner API
    void queueMove(float radiusMM, float thetaDegrees, bool absolute);
    void flushMoves();
    void endProgram();
    bool isPlannerFull() const;
    void feedHold();
    void resume();
    void abort();

    // Print driver status for debugging
    void printDriverStatus();

private:
    CoordinatedStepper  m_stepper;
    HomingManager       m_homingManager;
    TrajectoryPlanner   m_planner;

    TMC2208Stepper*    m_radiusDriver;
    TMC2208Stepper*    m_thetaDriver;

    bool  m_isHoming;
    bool  m_positionKnown;
    float m_feedrate;       // mm/min

    // Planner target position — tracks commanded position ahead of actual
    float m_plannerTargetR;     // mm
    float m_plannerTargetTheta; // degrees
};

#endif // MOTION_H
