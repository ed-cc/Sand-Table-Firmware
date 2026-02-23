// stepper_axis.h - Per-axis stepper state and port-register pin mapping
//
// Manages position counter, direction, enable state, and resolves Arduino
// pin numbers to direct PORT register pointers for single-cycle writes
// inside timer ISRs.

#ifndef STEPPER_AXIS_H
#define STEPPER_AXIS_H

#include <Arduino.h>
#include <stdint.h>

class StepperAxis {
public:
    StepperAxis();

    // Resolve pin numbers to PORT register pointers. Call once in setup().
    void init(uint8_t stepPin, uint8_t dirPin, uint8_t enPin);

    // Enable or disable the motor (LOW = enabled for TMC2208)
    void enable(bool on);

    // Set step direction: +1 forward, -1 reverse.
    // Writes the DIR pin immediately.
    void setDirection(int8_t dir);
    int8_t direction() const { return m_direction; }

    // Fire a single step pulse (HIGH then LOW).
    // For testing / slow moves only; the timer ISR uses port registers directly.
    void step();

    // Position counter (in microsteps)
    int32_t position() const { return m_position; }
    void setPosition(int32_t pos) { m_position = pos; }

    // Advance position by current direction (called from ISR after pulsing STEP)
    void advancePosition() { m_position += m_direction; }

    // Port register pointers and masks — public for direct ISR access.
    // Set by init(); NULL before init() is called.
    volatile uint8_t* stepPort;
    uint8_t           stepMask;
    volatile uint8_t* dirPort;
    uint8_t           dirMask;

    // Steps remaining in current move (managed by the move executor)
    int32_t stepsRemaining;

private:
    uint8_t  m_stepPin;
    uint8_t  m_dirPin;
    uint8_t  m_enPin;
    int8_t   m_direction;  // +1 or -1
    int32_t  m_position;   // current position in microsteps
};

#endif // STEPPER_AXIS_H
