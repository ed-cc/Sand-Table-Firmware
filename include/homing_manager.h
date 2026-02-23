// homing_manager.h - Endstop-based homing for TMC2208 with physical limit switches
//
// Implements a non-blocking state machine for homing each axis:
//   1. Move toward endstop at slow speed
//   2. On trigger: stop, back off
//   3. Approach again at very slow speed
//   4. On second trigger: stop and zero position
//
// The homing sequence is driven by calling run() repeatedly in the main loop.

#ifndef HOMING_MANAGER_H
#define HOMING_MANAGER_H

#include "coordinated_stepper.h"
#include <stdint.h>

// Homing state machine states
enum HomingState : uint8_t {
    HOMING_IDLE = 0,
    HOMING_RADIUS_FAST_APPROACH,
    HOMING_RADIUS_BACKOFF,
    HOMING_RADIUS_SLOW_APPROACH,
    HOMING_THETA_FAST_APPROACH,
    HOMING_THETA_BACKOFF,
    HOMING_THETA_SLOW_APPROACH,
    HOMING_COMPLETE
};

// Homing speeds and distances
static const float HOMING_FAST_SPEED_MM_MIN = 300.0f;   // Fast approach (mm/min equiv)
static const float HOMING_SLOW_SPEED_MM_MIN = 60.0f;    // Slow approach (mm/min equiv)
static const float HOMING_BACKOFF_MM = 3.0f;             // Back-off distance (mm)
static const float HOMING_BACKOFF_DEG = 3.0f;            // Back-off distance (degrees)

class HomingManager {
public:
    HomingManager();

    // Start the homing sequence. Non-blocking — call run() repeatedly.
    void start(CoordinatedStepper& stepper);

    // Advance the homing state machine. Call in main loop.
    // Returns true if homing is still in progress.
    bool run(CoordinatedStepper& stepper);

    // Check if homing is in progress
    bool isHoming() const { return m_state != HOMING_IDLE && m_state != HOMING_COMPLETE; }

    // Check if homing completed successfully
    bool isComplete() const { return m_state == HOMING_COMPLETE; }

    // Get current homing state (for debugging)
    HomingState state() const { return m_state; }

    // Called from endstop ISR — sets the triggered flag
    static void onRadiusEndstop();
    static void onThetaEndstop();

private:
    void setupEndstopInterrupts();

    HomingState m_state;

    // Endstop trigger flags (set by ISR, cleared by state machine)
    static volatile bool s_radiusEndstopTriggered;
    static volatile bool s_thetaEndstopTriggered;
};

#endif // HOMING_MANAGER_H
