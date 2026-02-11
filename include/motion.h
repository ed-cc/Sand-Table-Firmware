// motion.h - Motion control class for Sand Table
#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include "values.h"

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
    void enableMotors();
    void disableMotors();

    // Get current position in user units
    float getRadiusMM();
    float getThetaDegrees();

    // Print driver status for debugging
    void printDriverStatus();

private:
    // TMC2208 drivers using hardware serial
    TMC2208Stepper radiusDriver;
    TMC2208Stepper thetaDriver;

    // AccelStepper objects
    AccelStepper radiusStepper;
    AccelStepper thetaStepper;

    // Helper methods
    void setupTMCDrivers();
    bool testDriverCommunication();

    // Homing methods
    void homeTheta();
    void homeRadius();

    // Check endstop status
    void checkEndstops();

    // Convert between user units and steps
    long radiusMMToSteps(float mm);
    long thetaDegreesToSteps(float degrees);
    float radiusStepsToMM(long steps);
    float thetaStepsToDegrees(long steps);

    // Radius compensation for theta motion (mechanical linkage)
    long calculateRadiusCompensation(float thetaDegrees);
    long calculateRadiusCompensationSteps(long thetaSteps);

    // Member variables
    bool m_isHoming;
    bool m_isHomingTheta;
    bool m_isHomingRadius;
    bool m_motorsEnabled;
};

#endif // MOTION_H
