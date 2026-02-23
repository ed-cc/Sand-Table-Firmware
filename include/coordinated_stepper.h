// coordinated_stepper.h - Coordinated two-axis stepper motion controller
//
// Orchestrates StepperAxis, ThetaCompensation, SCurveProfile, and
// BresenhamSync into a unified polar move executor. This is the class
// that Motion wraps to provide the high-level motion API.
//
// Usage:
//   1. Call init() once in setup()
//   2. Call moveTo() or moveRelative() to start a move
//   3. Call stepTick() repeatedly (from timer ISR or polling loop)
//      to advance the move one dominant-axis step at a time
//   4. Check isComplete() to know when the move is done

#ifndef COORDINATED_STEPPER_H
#define COORDINATED_STEPPER_H

#include "stepper_axis.h"
#include "theta_compensation.h"
#include "scurve_profile.h"
#include "bresenham_sync.h"
#include "values.h"

class CoordinatedStepper {
public:
    CoordinatedStepper();

    // Initialise stepper axes with pin assignments from values.h.
    // Call once in setup(). Does NOT set up hardware timers (that's TimerBackend's job).
    void init();

    // Start an absolute move to the given position.
    // radiusMM:    target radius in mm
    // thetaDeg:    target theta in degrees
    // feedrate:    feedrate in mm/min (converted to steps/sec internally)
    void moveTo(float radiusMM, float thetaDeg, float feedrate);

    // Start a relative move from the current position.
    // deltaR:      radius change in mm
    // deltaTheta:  theta change in degrees
    // feedrate:    feedrate in mm/min
    void moveRelative(float deltaR, float deltaTheta, float feedrate);

    // Set current position without moving (for G92 and homing).
    void setPosition(float radiusMM, float thetaDeg);

    // Get current position in user units.
    float getRadiusMM() const;
    float getThetaDegrees() const;

    // True when no move is in progress.
    bool isComplete() const { return m_complete; }

    // Mark the current move as complete (called by timer backend when ISR finishes).
    void markComplete() { m_complete = true; }

    // Advance the move by one dominant-axis step.
    // Call from timer ISR or polling loop. Does nothing if no move is active.
    void stepTick();

    // Get current step index (for LUT-based interval lookup)
    int32_t currentStepIndex() const { return m_stepIndex; }

    // Get the S-curve profile (for timer backend to read intervals)
    const SCurveProfile& profile() const { return m_profile; }

    // Direct axis access (for timer backend ISR)
    StepperAxis& radiusAxis() { return m_radiusAxis; }
    StepperAxis& thetaAxis() { return m_thetaAxis; }

private:
    // Start a move given deltas in user units
    void startMove(float deltaR, float deltaTheta, float feedrate);

    StepperAxis       m_radiusAxis;
    StepperAxis       m_thetaAxis;
    ThetaCompensation m_compensation;
    SCurveProfile     m_profile;
    BresenhamSync     m_bresenham;

    // Current target position in user units (tracked for absolute moves)
    float m_targetRadiusMM;
    float m_targetThetaDeg;

    // Move state
    int32_t m_stepIndex;
    bool    m_complete;

    // Which physical axis is dominant/minor for current move
    bool m_thetaIsDominant;

    // Directions for current move
    int8_t m_thetaDir;
    int8_t m_radiusDir;
};

#endif // COORDINATED_STEPPER_H
