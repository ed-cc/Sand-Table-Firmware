// motion.cpp - Motion control wrapper implementation
//
// Wires TMC2208 UART driver configuration, CoordinatedStepper, TimerBackend,
// and HomingManager into the Motion public API consumed by MotionCommander.

#include "motion.h"
#include "timer_backend.h"
#include "values.h"
#include <math.h>

// TMC2208 drivers on hardware serial ports
// Radius: Serial3 (TX=14, RX=15)
// Theta:  Serial1 (TX=18, RX=19)
static TMC2208Stepper s_radiusDriver(&Serial3, R_SENSE);
static TMC2208Stepper s_thetaDriver(&Serial1, R_SENSE);

Motion::Motion()
    : m_radiusDriver(&s_radiusDriver)
    , m_thetaDriver(&s_thetaDriver)
    , m_isHoming(false)
    , m_positionKnown(false)
    , m_feedrate(1000.0f)  // Default 1000 mm/min
    , m_plannerTargetR(0.0f)
    , m_plannerTargetTheta(0.0f)
{
}

void Motion::setupSteppers() {
    // Initialise TMC2208 UART communication
    Serial3.begin(TMC_UART_BAUD);
    Serial1.begin(TMC_UART_BAUD);

    // Configure radius driver
    m_radiusDriver->begin();
    m_radiusDriver->pdn_disable(true);      // Use PDN pin for UART, not power-down
    m_radiusDriver->I_scale_analog(false);   // Current set by registers, not VREF pin
    m_radiusDriver->rms_current(RADIUS_MOTOR_CURRENT);
    m_radiusDriver->ihold(8);               // ~25% hold current
    m_radiusDriver->iholddelay(8);           // Smooth ramp transition
    m_radiusDriver->TPOWERDOWN(20);          // ~0.5s before hold current engages
    m_radiusDriver->microsteps(MICROSTEPS);
    m_radiusDriver->en_spreadCycle(RADIUS_SPREAD_CYCLE);
    m_radiusDriver->toff(3);                 // Enable driver output stage

    // Configure theta driver
    m_thetaDriver->begin();
    m_thetaDriver->pdn_disable(true);
    m_thetaDriver->I_scale_analog(false);
    m_thetaDriver->rms_current(THETA_MOTOR_CURRENT);
    m_thetaDriver->ihold(6);                // ~20% hold current (less gravity load)
    m_thetaDriver->iholddelay(8);
    m_thetaDriver->TPOWERDOWN(40);           // ~1s delay before hold
    m_thetaDriver->microsteps(MICROSTEPS);
    m_thetaDriver->en_spreadCycle(THETA_SPREAD_CYCLE);
    m_thetaDriver->toff(3);

    // Initialise stepper axes (pin configuration, port register resolution)
    m_stepper.init();

    // Initialise hardware timers
    timerBackendInit();

    // Initialise trajectory planner with axis pointers
    m_planner.init(&m_stepper.thetaAxis(), &m_stepper.radiusAxis());

    // Start with motors disabled
    disableMotors();

    Serial.println(F("Motion: TMC2208 drivers configured"));
    Serial.print(F("Motion: Radius "));
    Serial.print(RADIUS_MOTOR_CURRENT);
    Serial.print(F("mA, Theta "));
    Serial.print(THETA_MOTOR_CURRENT);
    Serial.println(F("mA"));

#ifdef DEBUG
    // Read back actual microstep config from drivers to verify UART took effect
    uint16_t rMicro = m_radiusDriver->microsteps();
    uint16_t tMicro = m_thetaDriver->microsteps();
    Serial.print(F("[DBG] Radius microsteps readback: "));
    Serial.println(rMicro);
    Serial.print(F("[DBG] Theta microsteps readback: "));
    Serial.println(tMicro);
    Serial.print(F("[DBG] Expected microsteps: "));
    Serial.println(MICROSTEPS);
    if (rMicro != MICROSTEPS || tMicro != MICROSTEPS) {
        Serial.println(F("[DBG] WARNING: Microstep mismatch! UART config may have failed."));
    }

    // Print key constants for verification
    Serial.print(F("[DBG] THETA_STEPS_PER_DEGREE="));
    Serial.println(THETA_STEPS_PER_DEGREE, 4);
    Serial.print(F("[DBG] THETA_DRIVE_RATIO="));
    Serial.println(THETA_DRIVE_RATIO, 4);
    Serial.print(F("[DBG] RADIUS_STEPS_PER_MM="));
    Serial.println(RADIUS_STEPS_PER_MM, 4);
    Serial.print(F("[DBG] RADIUS_CENTRE_STEPS_PER_DEGREE="));
    Serial.println(RADIUS_CENTRE_STEPS_PER_DEGREE, 4);
    Serial.print(F("[DBG] RADIUS_COMPENSATION_PER_THETA_STEP="));
    Serial.println(RADIUS_COMPENSATION_PER_THETA_STEP, 6);
#endif
}

void Motion::home() {
    Serial.println(F("[MOT] home() starting"));
    m_isHoming = true;
    enableMotors();
    m_homingManager.start(m_stepper);
}

void Motion::moveTo(float radiusMM, float thetaDegrees) {
    // Ensure motors are enabled before moving
    enableMotors();

    // Clamp to position limits
    if (radiusMM < RADIUS_MIN_MM) radiusMM = RADIUS_MIN_MM;
    if (radiusMM > RADIUS_MAX_MM) radiusMM = RADIUS_MAX_MM;

    Serial.print(F("[MOT] moveTo R="));
    Serial.print(radiusMM, 2);
    Serial.print(F(" A="));
    Serial.print(thetaDegrees, 2);
    Serial.print(F(" F="));
    Serial.print(m_feedrate, 1);
    Serial.print(F(" curR="));
    Serial.print(getRadiusMM(), 2);
    Serial.print(F(" curA="));
    Serial.println(getThetaDegrees(), 2);

    m_stepper.moveTo(radiusMM, thetaDegrees, m_feedrate);

    Serial.print(F("[MOT] after plan: complete="));
    Serial.print(m_stepper.isComplete() ? F("Y") : F("N"));
    Serial.print(F(" profSteps="));
    Serial.print(m_stepper.profile().totalSteps());
    Serial.print(F(" profTime="));
    Serial.println(m_stepper.profile().totalTime(), 4);

    timerBackendStartMove(m_stepper);
}

void Motion::moveRelative(float radiusMM, float thetaDegrees) {
    // Ensure motors are enabled before moving
    enableMotors();

    // Compute absolute target and clamp
    float targetR = getRadiusMM() + radiusMM;
    if (targetR < RADIUS_MIN_MM) targetR = RADIUS_MIN_MM;
    if (targetR > RADIUS_MAX_MM) targetR = RADIUS_MAX_MM;

    float deltaR = targetR - getRadiusMM();

    Serial.print(F("[MOT] moveRel dR="));
    Serial.print(deltaR, 2);
    Serial.print(F(" dA="));
    Serial.print(thetaDegrees, 2);
    Serial.print(F(" F="));
    Serial.print(m_feedrate, 1);
    Serial.print(F(" curR="));
    Serial.print(getRadiusMM(), 2);
    Serial.print(F(" curA="));
    Serial.println(getThetaDegrees(), 2);

    m_stepper.moveRelative(deltaR, thetaDegrees, m_feedrate);

    Serial.print(F("[MOT] after plan: complete="));
    Serial.print(m_stepper.isComplete() ? F("Y") : F("N"));
    Serial.print(F(" profSteps="));
    Serial.print(m_stepper.profile().totalSteps());
    Serial.print(F(" profTime="));
    Serial.println(m_stepper.profile().totalTime(), 4);

    timerBackendStartMove(m_stepper);
}

void Motion::queueMove(float radiusMM, float thetaDegrees, bool absolute) {
    enableMotors();

    float deltaR, deltaTheta;

    if (absolute) {
        // Clamp target radius
        if (radiusMM < RADIUS_MIN_MM) radiusMM = RADIUS_MIN_MM;
        if (radiusMM > RADIUS_MAX_MM) radiusMM = RADIUS_MAX_MM;

        deltaR = radiusMM - m_plannerTargetR;
        deltaTheta = thetaDegrees - m_plannerTargetTheta;

        m_plannerTargetR = radiusMM;
        m_plannerTargetTheta = thetaDegrees;
    } else {
        // Relative: clamp resulting radius
        float targetR = m_plannerTargetR + radiusMM;
        if (targetR < RADIUS_MIN_MM) targetR = RADIUS_MIN_MM;
        if (targetR > RADIUS_MAX_MM) targetR = RADIUS_MAX_MM;

        deltaR = targetR - m_plannerTargetR;
        deltaTheta = thetaDegrees;

        m_plannerTargetR = targetR;
        m_plannerTargetTheta += thetaDegrees;
    }

    float feed_mmps = m_feedrate / 60.0f;

    m_planner.queue_move(deltaR, deltaTheta, feed_mmps, m_plannerTargetR - deltaR);
}

void Motion::flushMoves() {
    m_planner.end_of_program();
    while (!m_planner.is_idle()) {
        m_planner.run();
    }
}

void Motion::endProgram() {
    m_planner.end_of_program();
}

bool Motion::isPlannerFull() const {
    return m_planner.buffer_full();
}

void Motion::run() {
    // Homing state machine
    if (m_isHoming) {
        // During homing, stepTick is called from main loop (no timer ISR for homing moves)
        m_stepper.stepTick();

        if (!m_homingManager.run(m_stepper)) {
            m_isHoming = false;
            if (m_homingManager.isComplete()) {
                m_positionKnown = true;
                m_plannerTargetR = getRadiusMM();
                m_plannerTargetTheta = getThetaDegrees();
                Serial.println(F("Homing complete"));
            } else {
                Serial.println(F("Homing failed"));
            }
        }
        return;
    }

    // Trajectory planner state management
    m_planner.run();

    // Normal operation: refill LUT for ISR-driven single-move stepping
    if (!timerBackendIsComplete()) {
        timerBackendRefillLUT(m_stepper);

        // Periodic ISR diagnostic (every ~1 second at 16MHz loop rate)
        static uint32_t lastDiagMillis = 0;
        uint32_t now = millis();
        if (now - lastDiagMillis >= 1000) {
            lastDiagMillis = now;
            Serial.print(F("[MOT] ISR fires="));
            Serial.print(timerBackendISRCount());
            Serial.print(F(" remaining="));
            Serial.println(timerBackendStepsRemaining());
        }
    } else if (!m_stepper.isComplete()) {
        // ISR finished all steps — mark stepper complete so the next command can run
        m_stepper.markComplete();
        Serial.print(F("[MOT] move complete, ISR fires="));
        Serial.println(timerBackendISRCount());
    }
}

bool Motion::isMovementComplete() {
    if (m_isHoming) return false;
    return m_stepper.isComplete() && timerBackendIsComplete() && m_planner.is_idle();
}

void Motion::enableMotors(bool radius, bool theta) {
    Serial.print(F("[MOT] enableMotors R="));
    Serial.print(radius ? F("Y") : F("N"));
    Serial.print(F(" T="));
    Serial.println(theta ? F("Y") : F("N"));
    if (radius) m_stepper.radiusAxis().enable(true);
    if (theta) m_stepper.thetaAxis().enable(true);
}

void Motion::disableMotors(bool radius, bool theta) {
    if (radius) m_stepper.radiusAxis().enable(false);
    if (theta) m_stepper.thetaAxis().enable(false);
    timerBackendStop();
}

float Motion::getRadiusMM() {
    return m_stepper.getRadiusMM();
}

float Motion::getThetaDegrees() {
    return m_stepper.getThetaDegrees();
}

void Motion::setPosition(float radiusMM, float thetaDegrees) {
    Serial.print(F("[MOT] setPosition R="));
    Serial.print(radiusMM, 2);
    Serial.print(F(" A="));
    Serial.println(thetaDegrees, 2);
    m_stepper.setPosition(radiusMM, thetaDegrees);
    m_plannerTargetR = radiusMM;
    m_plannerTargetTheta = thetaDegrees;
    m_positionKnown = true;
}

void Motion::feedHold() {
    m_planner.feed_hold();
}

void Motion::resume() {
    m_planner.resume();
}

void Motion::abort() {
    m_planner.abort();
}

void Motion::printDriverStatus() {
    Serial.println(F("=== TMC2208 Driver Status ==="));

    Serial.println(F("-- Radius Driver --"));
    Serial.print(F("  Enabled: "));
    Serial.println(m_radiusDriver->isEnabled() ? F("yes") : F("no"));
    Serial.print(F("  Current (mA): "));
    Serial.println(m_radiusDriver->rms_current());
    Serial.print(F("  Microsteps: "));
    Serial.println(m_radiusDriver->microsteps());
    Serial.print(F("  StealthChop: "));
    Serial.println(!m_radiusDriver->en_spreadCycle() ? F("yes") : F("no"));

    Serial.println(F("-- Theta Driver --"));
    Serial.print(F("  Enabled: "));
    Serial.println(m_thetaDriver->isEnabled() ? F("yes") : F("no"));
    Serial.print(F("  Current (mA): "));
    Serial.println(m_thetaDriver->rms_current());
    Serial.print(F("  Microsteps: "));
    Serial.println(m_thetaDriver->microsteps());
    Serial.print(F("  StealthChop: "));
    Serial.println(!m_thetaDriver->en_spreadCycle() ? F("yes") : F("no"));

    Serial.println(F("-- Position --"));
    Serial.print(F("  Radius: "));
    Serial.print(getRadiusMM(), 2);
    Serial.println(F(" mm"));
    Serial.print(F("  Theta: "));
    Serial.print(getThetaDegrees(), 2);
    Serial.println(F(" deg"));
}
