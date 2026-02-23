// stepper_axis.cpp - Per-axis stepper state and port-register pin mapping

#include "stepper_axis.h"

StepperAxis::StepperAxis()
    : stepPort(nullptr)
    , stepMask(0)
    , dirPort(nullptr)
    , dirMask(0)
    , stepsRemaining(0)
    , m_stepPin(0)
    , m_dirPin(0)
    , m_enPin(0)
    , m_direction(1)
    , m_position(0)
{
}

void StepperAxis::init(uint8_t stepPin, uint8_t dirPin, uint8_t enPin) {
    m_stepPin = stepPin;
    m_dirPin  = dirPin;
    m_enPin   = enPin;

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);

    // Start disabled (HIGH = disabled for TMC2208)
    digitalWrite(m_enPin, HIGH);

#ifndef UNIT_TEST
    // Resolve Arduino pin numbers to PORT register pointers for direct writes.
    // digitalPinToPort() and portOutputRegister() are AVR core macros that
    // perform a compile-time table lookup — safe in setup(), zero cost in ISR.
    uint8_t stepPortIdx = digitalPinToPort(stepPin);
    stepPort = portOutputRegister(stepPortIdx);
    stepMask = digitalPinToBitMask(stepPin);

    uint8_t dirPortIdx = digitalPinToPort(dirPin);
    dirPort = portOutputRegister(dirPortIdx);
    dirMask = digitalPinToBitMask(dirPin);
#else
    // In unit test mode, port registers are not available.
    // Set non-null dummy values so tests can verify init was called.
    static volatile uint8_t dummyPort = 0;
    stepPort = &dummyPort;
    stepMask = 1;
    dirPort  = &dummyPort;
    dirMask  = 1;
#endif
}

void StepperAxis::enable(bool on) {
    // TMC2208: LOW = enabled, HIGH = disabled
    digitalWrite(m_enPin, on ? LOW : HIGH);
}

void StepperAxis::setDirection(int8_t dir) {
    m_direction = dir;
    digitalWrite(m_dirPin, dir > 0 ? HIGH : LOW);
}

void StepperAxis::step() {
    digitalWrite(m_stepPin, HIGH);
    delayMicroseconds(2);  // Minimum pulse width for TMC2208 (100ns min, 2us safe)
    digitalWrite(m_stepPin, LOW);
    m_position += m_direction;
}
