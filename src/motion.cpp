// motion.cpp - Motion control implementation for Sand Table
#include "motion.h"

Motion::Motion()
    : radiusDriver(&Serial3, R_SENSE),
      thetaDriver(&Serial1, R_SENSE),
      radiusStepper(AccelStepper::DRIVER, RADIUS_STEP_PIN, RADIUS_DIR_PIN),
      thetaStepper(AccelStepper::DRIVER, THETA_STEP_PIN, THETA_DIR_PIN),
      m_isHoming(false),
      m_isHomingTheta(false),
      m_isHomingRadius(false),
      m_motorsEnabled(false)
{
}

void Motion::setupSteppers()
{
    Serial.println(F("\nInitializing motion control system..."));
    Serial.println(F("======================================"));

    // Initialize endstop pins
    pinMode(RADIUS_ENDSTOP_PIN, RADIUS_ENDSTOP_PULLUP ? INPUT_PULLUP : INPUT);
    pinMode(THETA_ENDSTOP_PIN, THETA_ENDSTOP_PULLUP ? INPUT_PULLUP : INPUT);
    Serial.println(F("✓ Endstop pins initialized"));

    // Initialize motor enable pins (CRITICAL - was missing!)
    pinMode(RADIUS_EN_PIN, OUTPUT);
    pinMode(THETA_EN_PIN, OUTPUT);
    Serial.println(F("✓ Enable pins initialized"));

    // Initialize hardware serial ports for TMC drivers
    Serial1.begin(TMC_UART_BAUD);  // Theta motor (pins 18 TX, 19 RX)
    Serial3.begin(TMC_UART_BAUD);  // Radius motor (pins 14 TX, 15 RX)
    disableMotors(); // Disable during setup

    delay(100); // Allow drivers to power up

    // Configure TMC2208 drivers
    Serial.println(F("Configuring TMC2208 drivers..."));
    setupTMCDrivers();

    // Test communication
    Serial.println(F("\nTesting driver communication..."));
    if (testDriverCommunication())
    {
        Serial.println(F("✓ Driver communication successful!"));
    }
    else
    {
        Serial.println(F("✗ Warning: Driver communication issues detected"));
        Serial.println(F("  Check wiring and R_SENSE value in values.h"));
    }

    // Configure AccelStepper
    Serial.println(F("\nConfiguring motion parameters..."));

    // Radius motor
    radiusStepper.setMaxSpeed(RADIUS_MAX_SPEED);
    radiusStepper.setAcceleration(RADIUS_ACCELERATION);
    radiusStepper.setCurrentPosition(0);

    // Theta motor
    thetaStepper.setMaxSpeed(THETA_MAX_SPEED);
    thetaStepper.setAcceleration(THETA_ACCELERATION);
    thetaStepper.setCurrentPosition(0);

    // Enable drivers
    enableMotors();

    printDriverStatus();

    Serial.println(F("\n======================================"));
    Serial.println(F("Motion system initialized!"));
    Serial.println(F("======================================\n"));
}

void Motion::home()
{
    Serial.println(F("Homing to (0mm, 0°)..."));
    homeTheta();
    m_isHoming = true;
}

void Motion::moveTo(float radiusMM, float thetaDegrees)
{
    long radiusSteps = radiusMMToSteps(radiusMM);
    long thetaSteps = thetaDegreesToSteps(thetaDegrees);

    // Calculate radius compensation for theta position (mechanical linkage)
    long radiusCompensation = calculateRadiusCompensation(thetaDegrees);

    // Set motor positions (radius includes compensation for theta)
    radiusStepper.moveTo(radiusSteps + radiusCompensation);
    thetaStepper.moveTo(thetaSteps);

    Serial.print(F("Moving to: R="));
    Serial.print(radiusMM);
    Serial.print(F("mm, Θ="));
    Serial.print(thetaDegrees);
    Serial.println(F("°"));
}

void Motion::moveRelative(float radiusMM, float thetaDegrees)
{
    long radiusSteps = radiusMMToSteps(radiusMM);
    long thetaSteps = thetaDegreesToSteps(thetaDegrees);

    // Calculate radius compensation for the theta change (mechanical linkage)
    long radiusCompensation = calculateRadiusCompensation(thetaDegrees);

    // Move motors (radius includes compensation for theta change)
    radiusStepper.move(radiusSteps + radiusCompensation);
    thetaStepper.move(thetaSteps);

    Serial.print(F("Moving relative: ΔR="));
    Serial.print(radiusMM);
    Serial.print(F("mm, ΔΘ="));
    Serial.print(thetaDegrees);
    Serial.println(F("°"));
}

void Motion::run()
{
    checkEndstops();
    radiusStepper.run();
    thetaStepper.run();
}

bool Motion::isMovementComplete()
{
    return !radiusStepper.isRunning() && !thetaStepper.isRunning();
}

void Motion::enableMotors()
{
    digitalWrite(RADIUS_EN_PIN, LOW);
    digitalWrite(THETA_EN_PIN, LOW);
    Serial.println(F("✓ Motors enabled"));
}

void Motion::disableMotors()
{
    digitalWrite(RADIUS_EN_PIN, HIGH);
    digitalWrite(THETA_EN_PIN, HIGH);
    Serial.println(F("Motors disabled"));
}

float Motion::getRadiusMM()
{
    return radiusStepsToMM(radiusStepper.currentPosition());
}

float Motion::getThetaDegrees()
{
    return thetaStepsToDegrees(thetaStepper.currentPosition());
}

void Motion::printDriverStatus()
{
    Serial.println(F("\nDriver Status:"));

    // Radius driver
    Serial.print(F("  Radius - "));
    if (radiusDriver.ot())
        Serial.print(F("OVERTEMP! "));
    if (radiusDriver.otpw())
        Serial.print(F("TEMP_WARNING "));
    if (radiusDriver.s2ga() || radiusDriver.s2gb())
        Serial.print(F("SHORT_GND! "));
    Serial.print(F("Standstill: "));
    Serial.println(radiusDriver.stst() ? F("Yes") : F("No"));

    // Theta driver
    Serial.print(F("  Theta  - "));
    if (thetaDriver.ot())
        Serial.print(F("OVERTEMP! "));
    if (thetaDriver.otpw())
        Serial.print(F("TEMP_WARNING "));
    if (thetaDriver.s2ga() || thetaDriver.s2gb())
        Serial.print(F("SHORT_GND! "));
    Serial.print(F("Standstill: "));
    Serial.println(thetaDriver.stst() ? F("Yes") : F("No"));
}

// ===== Private Helper Methods =====

void Motion::setupTMCDrivers()
{
    // === Radius Motor Configuration ===
    radiusDriver.begin();                             // Initialize driver
    radiusDriver.mstep_reg_select(true);              // Use MSTEP register for microstepping
    radiusDriver.I_scale_analog(false);               // Use internal current reference
    radiusDriver.toff(4);                             // Enable driver (off time = 4)
    radiusDriver.rms_current(RADIUS_MOTOR_CURRENT);   // Set motor current
    radiusDriver.microsteps(MICROSTEPS);              // Set microstepping
    radiusDriver.en_spreadCycle(RADIUS_SPREAD_CYCLE); // Chopper mode
    radiusDriver.pwm_autoscale(true);                 // Enable automatic current scaling
    radiusDriver.TPOWERDOWN(128);                     // ~2 seconds standstill before power down

    // === Theta Motor Configuration ===
    thetaDriver.begin();                              // Initialize driver
    thetaDriver.mstep_reg_select(true);               // Use MSTEP register for microstepping
    thetaDriver.I_scale_analog(false);                // Use internal current reference
    thetaDriver.toff(4);
    thetaDriver.rms_current(THETA_MOTOR_CURRENT);
    thetaDriver.microsteps(MICROSTEPS);
    thetaDriver.en_spreadCycle(THETA_SPREAD_CYCLE);
    thetaDriver.pwm_autoscale(true);
    thetaDriver.TPOWERDOWN(128);

    Serial.println(F("✓ TMC2208 registers configured"));
    Serial.print(F("  Radius motor: "));
    Serial.print(RADIUS_MOTOR_CURRENT);
    Serial.println(F("mA RMS"));
    Serial.print(F("  Theta motor:  "));
    Serial.print(THETA_MOTOR_CURRENT);
    Serial.println(F("mA RMS"));
    Serial.print(F("  Microstepping: 1/"));
    Serial.println(MICROSTEPS);
    Serial.print(F("  Mode: "));
    Serial.println(RADIUS_SPREAD_CYCLE ? F("SpreadCycle") : F("StealthChop (silent)"));
}

bool Motion::testDriverCommunication()
{
    bool radiusOK = false;
    bool thetaOK = false;

    // Test radius driver
    Serial.print(F("  Radius (Serial3 - TX=14, RX=15): "));

    uint32_t radiusGCONF = radiusDriver.GCONF();
    if (radiusGCONF != 0 && radiusGCONF != 0xFFFFFFFF)
    {
        radiusOK = true;
        Serial.print(F("✓ GCONF=0x"));
        Serial.println(radiusGCONF, HEX);
    }
    else
    {
        Serial.print(F("✗ No response (GCONF=0x"));
        Serial.print(radiusGCONF, HEX);
        Serial.println(F(")"));
    }

    // Test theta driver
    Serial.print(F("  Theta (Serial1 - TX=18, RX=19): "));

    uint32_t thetaGCONF = thetaDriver.GCONF();
    if (thetaGCONF != 0 && thetaGCONF != 0xFFFFFFFF)
    {
        thetaOK = true;
        Serial.print(F("✓ GCONF=0x"));
        Serial.println(thetaGCONF, HEX);
    }
    else
    {
        Serial.print(F("✗ No response (GCONF=0x"));
        Serial.print(thetaGCONF, HEX);
        Serial.println(F(")"));
    }

    if (!radiusOK || !thetaOK)
    {
        Serial.println(F("\n  Troubleshooting steps:"));
        if (!radiusOK)
        {
            Serial.println(F("  Radius driver:"));
            Serial.println(F("    - Verify Serial3 pins (TX=14, RX=15) → driver PDN_UART pin"));
            Serial.println(F("    - Check MS1/MS2 pins are HIGH (floating or tied to VCC)"));
            Serial.println(F("    - Verify driver has power (VM and VIO)"));
        }
        if (!thetaOK)
        {
            Serial.println(F("  Theta driver:"));
            Serial.println(F("    - Verify Serial1 pins (TX=18, RX=19) → driver PDN_UART pin"));
            Serial.println(F("    - Check MS1/MS2 pins are HIGH (floating or tied to VCC)"));
            Serial.println(F("    - Verify driver has power (VM and VIO)"));
            Serial.println(F("    - Try swapping driver positions to test if hardware faulty"));
        }
    }

    return radiusOK && thetaOK;
}

void Motion::checkEndstops()
{
    if (!m_isHoming)
        return;

    bool radiusTriggered = digitalRead(RADIUS_ENDSTOP_PIN) == LOW;
    bool thetaTriggered = digitalRead(THETA_ENDSTOP_PIN) == LOW;

    if (thetaTriggered)
    {
        thetaStepper.stop();
        Serial.println(F("⚠ Theta endstop triggered!"));
        if (m_isHomingTheta)
        {
            m_isHomingTheta = false;
            homeRadius();
        }
    }

    if (radiusTriggered)
    {
        radiusStepper.stop();
        Serial.println(F("⚠ Radius endstop triggered!"));
        if (m_isHomingRadius)
        {
            m_isHomingRadius = false;
            m_isHoming = false;
            Serial.println(F("✓ Homing complete!"));
        }
    }
}

void Motion::homeTheta()
{
    Serial.println(F("Homing theta..."));
    long thetaSteps = -THETA_MAX_HOMING_DEGREES * THETA_STEPS_PER_DEGREE;
    long radiusCompensation = calculateRadiusCompensationSteps(thetaSteps);
    thetaStepper.move(thetaSteps);
    radiusStepper.move(radiusCompensation);
    m_isHomingTheta = true;
}

void Motion::homeRadius()
{
    Serial.println(F("Homing radius..."));
    radiusStepper.move(-RADIUS_MAX_HOMING_MM * RADIUS_STEPS_PER_MM);
    m_isHomingRadius = true;
}

// ===== Unit Conversion Methods =====

long Motion::radiusMMToSteps(float mm)
{
    return (long)(mm * RADIUS_STEPS_PER_MM);
}

long Motion::thetaDegreesToSteps(float degrees)
{
    return (long)(degrees * THETA_STEPS_PER_DEGREE);
}

float Motion::radiusStepsToMM(long steps)
{
    return (float)steps / RADIUS_STEPS_PER_MM;
}

float Motion::thetaStepsToDegrees(long steps)
{
    return (float)steps / THETA_STEPS_PER_DEGREE;
}

// ===== Radius Compensation Methods =====

long Motion::calculateRadiusCompensation(float thetaDegrees)
{
    // Calculate how many steps the radius motor needs to move
    // to compensate for theta rotation and maintain constant radius
    return (long)(thetaDegrees * RADIUS_STEPS_PER_THETA_DEGREE);
}

long Motion::calculateRadiusCompensationSteps(long thetaSteps)
{
    // Calculate radius compensation from theta steps
    // Convert theta steps to degrees first, then calculate compensation
    float thetaDegrees = thetaStepsToDegrees(thetaSteps);
    return calculateRadiusCompensation(thetaDegrees);
}
