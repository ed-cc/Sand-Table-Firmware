// coordinated_stepper.cpp - Coordinated two-axis stepper motion controller

#include "coordinated_stepper.h"
#include <math.h>

CoordinatedStepper::CoordinatedStepper()
    : m_targetRadiusMM(0)
    , m_targetThetaDeg(0)
    , m_stepIndex(0)
    , m_complete(true)
    , m_thetaIsDominant(true)
    , m_thetaDir(1)
    , m_radiusDir(1)
{
}

void CoordinatedStepper::init() {
    m_thetaAxis.init(THETA_STEP_PIN, THETA_DIR_PIN, THETA_EN_PIN);
    m_radiusAxis.init(RADIUS_STEP_PIN, RADIUS_DIR_PIN, RADIUS_EN_PIN);
    m_complete = true;
    m_targetRadiusMM = 0;
    m_targetThetaDeg = 0;
}

void CoordinatedStepper::moveTo(float radiusMM, float thetaDeg, float feedrate) {
    float deltaR = radiusMM - getRadiusMM();
    float deltaTheta = thetaDeg - getThetaDegrees();
    m_targetRadiusMM = radiusMM;
    m_targetThetaDeg = thetaDeg;
    startMove(deltaR, deltaTheta, feedrate);
}

void CoordinatedStepper::moveRelative(float deltaR, float deltaTheta, float feedrate) {
    m_targetRadiusMM = getRadiusMM() + deltaR;
    m_targetThetaDeg = getThetaDegrees() + deltaTheta;
    startMove(deltaR, deltaTheta, feedrate);
}

void CoordinatedStepper::setPosition(float radiusMM, float thetaDeg) {
    m_thetaAxis.setPosition((int32_t)roundf(thetaDeg * THETA_STEPS_PER_DEGREE));
    // Radius motor position = commanded radius steps + compensation for current theta
    float compensationSteps = thetaDeg * RADIUS_CENTRE_STEPS_PER_DEGREE;
    m_radiusAxis.setPosition((int32_t)roundf(radiusMM * RADIUS_STEPS_PER_MM + compensationSteps));
    m_targetRadiusMM = radiusMM;
    m_targetThetaDeg = thetaDeg;
}

float CoordinatedStepper::getRadiusMM() const {
    // Radius motor position includes both commanded radius AND theta compensation.
    // Subtract compensation to get the true commanded radius.
    float thetaDeg = getThetaDegrees();
    float compensationSteps = thetaDeg * RADIUS_CENTRE_STEPS_PER_DEGREE;
    return ((float)m_radiusAxis.position() - compensationSteps) / RADIUS_STEPS_PER_MM;
}

float CoordinatedStepper::getThetaDegrees() const {
    return (float)m_thetaAxis.position() / THETA_STEPS_PER_DEGREE;
}

void CoordinatedStepper::startMove(float deltaR, float deltaTheta, float feedrate) {
    // Decompose into step counts (with theta compensation)
    MoveSteps steps = ThetaCompensation::decompose(deltaR, deltaTheta);

    int32_t absTheta = (steps.thetaSteps >= 0) ? steps.thetaSteps : -steps.thetaSteps;
    int32_t absRadius = (steps.radiusSteps >= 0) ? steps.radiusSteps : -steps.radiusSteps;

    Serial.print(F("[CS] startMove dR="));
    Serial.print(deltaR, 2);
    Serial.print(F(" dA="));
    Serial.print(deltaTheta, 2);
    Serial.print(F(" -> thetaSteps="));
    Serial.print(steps.thetaSteps);
    Serial.print(F(" radiusSteps="));
    Serial.print(steps.radiusSteps);
    Serial.print(F(" (abs T="));
    Serial.print(absTheta);
    Serial.print(F(" R="));
    Serial.print(absRadius);
    Serial.println(F(")"));

    // Handle zero-length move
    if (absTheta == 0 && absRadius == 0) {
        Serial.println(F("[CS] zero-length move, skipping"));
        m_complete = true;
        return;
    }

    // Set directions
    m_thetaDir = (steps.thetaSteps >= 0) ? 1 : -1;
    m_radiusDir = (steps.radiusSteps >= 0) ? 1 : -1;
    m_thetaAxis.setDirection(m_thetaDir);
    m_radiusAxis.setDirection(m_radiusDir);

    // Set steps remaining
    m_thetaAxis.stepsRemaining = absTheta;
    m_radiusAxis.stepsRemaining = absRadius;

    // Determine dominant axis for Bresenham sync
    m_bresenham.init(absTheta, absRadius);
    m_thetaIsDominant = m_bresenham.isADominant();

    // Convert feedrate (mm/min) to dominant axis steps/sec
    // Feedrate is the tangential speed of the ball bearing.
    // For simplicity, we use the dominant axis's steps-per-unit conversion.
    float feedMmPerSec = feedrate / 60.0f;
    float maxVelStepsPerSec;
    float maxAccel;
    float jerk;

    if (m_thetaIsDominant) {
        // Theta dominant: convert mm/s to deg/s, then to steps/s
        // Approximate: use a nominal radius for conversion, or just use theta limits
        maxVelStepsPerSec = (float)THETA_MAX_SPEED;
        maxAccel = (float)THETA_ACCELERATION;
        jerk = THETA_JERK;

        // Apply feedrate limit: convert mm/s to steps/s via theta
        float feedStepsPerSec = feedMmPerSec * THETA_STEPS_PER_DEGREE / 1.0f;
        // Rough: 1 degree at radius R mm is approx R*pi/180 mm of arc
        // But we don't know R here, so just cap at axis max
        if (feedStepsPerSec < maxVelStepsPerSec && feedStepsPerSec > 0) {
            maxVelStepsPerSec = feedStepsPerSec;
        }
    } else {
        // Radius dominant
        maxVelStepsPerSec = (float)RADIUS_MAX_SPEED;
        maxAccel = (float)RADIUS_ACCELERATION;
        jerk = RADIUS_JERK;

        float feedStepsPerSec = feedMmPerSec * RADIUS_STEPS_PER_MM;
        if (feedStepsPerSec < maxVelStepsPerSec && feedStepsPerSec > 0) {
            maxVelStepsPerSec = feedStepsPerSec;
        }
    }

    // Plan S-curve profile for the dominant axis
    m_profile.plan(m_bresenham.totalDominant(), maxVelStepsPerSec, maxAccel, jerk);

    Serial.print(F("[CS] dom="));
    Serial.print(m_thetaIsDominant ? F("theta") : F("radius"));
    Serial.print(F(" domSteps="));
    Serial.print(m_bresenham.totalDominant());
    Serial.print(F(" minSteps="));
    Serial.print(m_bresenham.totalMinor());
    Serial.print(F(" Vmax="));
    Serial.print(maxVelStepsPerSec, 1);
    Serial.print(F(" Amax="));
    Serial.print(maxAccel, 1);
    Serial.print(F(" J="));
    Serial.println(jerk, 1);

    Serial.print(F("[CS] profile: totalSteps="));
    Serial.print(m_profile.totalSteps());
    Serial.print(F(" totalTime="));
    Serial.print(m_profile.totalTime(), 4);
    Serial.println(F("s"));

    m_stepIndex = 0;
    m_complete = false;
}

void CoordinatedStepper::stepTick() {
    if (m_complete) return;

    // Check if move is done
    if (m_bresenham.isComplete()) {
        m_complete = true;
        return;
    }

    // Step the dominant axis
    if (m_thetaIsDominant) {
        m_thetaAxis.step();
        m_thetaAxis.stepsRemaining--;
    } else {
        m_radiusAxis.step();
        m_radiusAxis.stepsRemaining--;
    }

    // Bresenham: check if minor axis also steps
    bool fireMinor = m_bresenham.stepMinor();
    if (fireMinor) {
        if (m_thetaIsDominant) {
            m_radiusAxis.step();
            m_radiusAxis.stepsRemaining--;
        } else {
            m_thetaAxis.step();
            m_thetaAxis.stepsRemaining--;
        }
    }

    m_stepIndex++;

    // Check completion after stepping
    if (m_bresenham.isComplete()) {
        m_complete = true;
    }
}
