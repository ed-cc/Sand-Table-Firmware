// values.h - Hardware configuration and mechanical constants
#ifndef VALUES_H
#define VALUES_H

// ===== TMC2208 Driver Configuration =====
// IMPORTANT: For UART mode to work, TMC2208 boards must be configured:
//   1. MS1 and MS2 must be HIGH (leave floating or connect to VCC via 10k-100k)
//   2. PDN_UART must be HIGH (usually connected to VCC on breakout boards)
//   3. Connect MCU TX → Driver RX (PDN_UART pin on TMC2208)
//   4. Connect MCU RX → Driver TX (if available, may not be on all boards)
//   5. INDEX pin not needed for basic operation
//
// R_SENSE value verification:
//   - Most BigTreeTech TMC2208 v3.0: 0.11Ω
//   - Some older versions: 0.12Ω
//   - Check the small resistor near driver chip (usually marked "R110" = 0.11Ω)
#define R_SENSE 0.11f  // R_SENSE resistor value (Ω) - verify on your board!

// Motor current settings (mA RMS)
#define RADIUS_MOTOR_CURRENT 1000  // Start conservative, can increase to 1200-1400
#define THETA_MOTOR_CURRENT 1000   // Start conservative, can increase to 1200-1400

// Chopper mode (false = StealthChop/silent, true = SpreadCycle/more torque)
#define RADIUS_SPREAD_CYCLE false  // Use StealthChop for quiet operation
#define THETA_SPREAD_CYCLE false   // Use StealthChop for quiet operation

// ===== Radius Motor ===== X on Ramps board
#define RADIUS_STEP_PIN 54  // A0
#define RADIUS_DIR_PIN 55   // A1
#define RADIUS_EN_PIN 38
// Hardware Serial3 (TX=14, RX=15) used for TMC2208 UART communication

// ===== Theta Motor ===== Y on Ramps board
#define THETA_STEP_PIN 60  // A6
#define THETA_DIR_PIN 61   // A7
#define THETA_EN_PIN 56
// Hardware Serial1 (TX=18, RX=19) used for TMC2208 UART communication

// ===== Mechanical Constants =====
// Microstepping configuration
#define MICROSTEPS 16  // Native microstepping (TMC2208 interpolates to 256)

// Stepper motor specifications
#define STEPS_PER_REV 200  // Standard 1.8° stepper (200 steps/revolution)

// ===== Drive Ratios =====
// Radius axis: 16:20 pulley ratio before belt drive
#define RADIUS_MOTOR_TEETH 16
#define RADIUS_INTERMEDIATE_TEETH 20
#define RADIUS_DRIVE_RATIO (RADIUS_INTERMEDIATE_TEETH / (float)RADIUS_MOTOR_TEETH)  // = 1.25

// Centre pulley (for theta compensation linkage)
#define RADIUS_CENTRE_PULLEY_TEETH 100  // Central pulley linked to theta rotation

// Theta axis: 16:210 pulley ratio
#define THETA_MOTOR_TEETH 16
#define THETA_OUTPUT_TEETH 210
#define THETA_DRIVE_RATIO (THETA_OUTPUT_TEETH / (float)THETA_MOTOR_TEETH)  // = 13.125

// ===== Belt Drive Configuration (Radius axis) =====
#define BELT_PITCH_MM 2.0f        // GT2 belt pitch (adjust if using different belt)
#define BELT_PULLEY_TEETH 16      // Teeth on the pulley that drives the belt

// ===== Steps per Unit Calculations =====
// Radius axis steps per mm (includes drive ratio and belt conversion)
// Formula: (STEPS_PER_REV * MICROSTEPS * RADIUS_DRIVE_RATIO) / (BELT_PULLEY_TEETH * BELT_PITCH_MM)
#define RADIUS_STEPS_PER_MM ((STEPS_PER_REV * MICROSTEPS * RADIUS_DRIVE_RATIO) / (BELT_PULLEY_TEETH * BELT_PITCH_MM))
// = (200 * 16 * 1.25) / (16 * 2) = 125 steps/mm

// Radius compensation for theta motion (reference frame translation)
//
// MECHANICAL CONSTRAINT: The 100T center pulley must rotate at the same angular
// velocity as the 210T theta output to prevent belt wind-up in the rotating reference frame.
//
// When theta motor moves 210 steps:
//   - Theta output (210T) rotates by: 210 * (16/210) = 16 teeth worth
//   - Radius compensation must rotate 100T center pulley by the same angle
//   - Required radius motor movement: 100 steps
//   - Compensation ratio: 100/210 = 0.47619 (or 10/21)
//
// Calculation breakdown:
//   - Theta motor drives 210T output at ratio: 210/16 = 13.125
//   - Radius motor drives 100T center via 16:20 intermediate: effective ratio for 100T rotation
//   - RADIUS_CENTRE_DRIVE_RATIO represents the effective ratio to rotate the 100T pulley
//   - Compensation per theta step: (100/210) = RADIUS_CENTRE_DRIVE_RATIO / THETA_DRIVE_RATIO
//
#define RADIUS_CENTRE_DRIVE_RATIO (RADIUS_CENTRE_PULLEY_TEETH / (float)RADIUS_MOTOR_TEETH)  // = 100/16 = 6.25
#define RADIUS_COMPENSATION_PER_THETA_STEP (RADIUS_CENTRE_DRIVE_RATIO / THETA_DRIVE_RATIO)
// = 6.25 / 13.125 = 0.47619 = 100/210 (verified: 210 theta steps → 100 radius comp steps)

// Radius motor steps per degree of centre pulley rotation
#define RADIUS_CENTRE_STEPS_PER_DEGREE ((STEPS_PER_REV * MICROSTEPS * RADIUS_CENTRE_DRIVE_RATIO) / 360.0f)
// = (200 * 16 * 6.25) / 360 = 55.556 steps/degree

// Theta axis steps per degree (includes drive ratio)
// Formula: (STEPS_PER_REV * MICROSTEPS * THETA_DRIVE_RATIO) / 360
#define THETA_STEPS_PER_DEGREE ((STEPS_PER_REV * MICROSTEPS * THETA_DRIVE_RATIO) / 360.0f)
// = (200 * 16 * 13.125) / 360 = 116.667 steps/degree

// ===== Motion Limits =====
// Maximum travel limits (in user units)
#define RADIUS_MIN_MM -290.0f
#define RADIUS_MAX_MM 290.0f

#define THETA_MIN_DEGREES 0.0f
#define THETA_MAX_DEGREES 360.0f

// Maximum distance to move when homing (to prevent runaway if endstops fail)
#define RADIUS_MAX_HOMING_MM 600.0f
#define THETA_MAX_HOMING_DEGREES 400.0f

// ===== Speed and Acceleration =====
// Maximum speeds (steps/second)
#define THETA_MAX_SPEED 2000    // Theta can typically move faster
#define RADIUS_MAX_SPEED 20000   // Adjust based on testing

// Acceleration (steps/second²)
#define THETA_ACCELERATION 1500   // Scaled to match speed (reaches max in ~1.3s)
#define RADIUS_ACCELERATION 10000  // Scaled to match speed (reaches max in ~1.3s)

// Jerk limits (steps/sec³) - rate of change of acceleration
// jerk = acceleration * 10 means ~0.1s to reach max accel
#define RADIUS_JERK 100000.0f   // steps/sec³
#define THETA_JERK 15000.0f     // steps/sec³

// Physical-unit convenience macros (for CoordinatedStepper API)
#define RADIUS_MAX_SPEED_MM_S   (RADIUS_MAX_SPEED / RADIUS_STEPS_PER_MM)
#define RADIUS_ACCEL_MM_S2      (RADIUS_ACCELERATION / RADIUS_STEPS_PER_MM)
#define RADIUS_JERK_MM_S3       (RADIUS_JERK / RADIUS_STEPS_PER_MM)
#define THETA_MAX_SPEED_DEG_S   (THETA_MAX_SPEED / THETA_STEPS_PER_DEGREE)
#define THETA_ACCEL_DEG_S2      (THETA_ACCELERATION / THETA_STEPS_PER_DEGREE)
#define THETA_JERK_DEG_S3       (THETA_JERK / THETA_STEPS_PER_DEGREE)

// ===== UART Configuration =====
#define TMC_UART_BAUD 57600  // Standard baudrate for TMC2208

// ===== Homing Configuration =====
// Endstop pins for homing
#define RADIUS_ENDSTOP_PIN 3   // Radius axis limit switch (X-MIN on RAMPS)
#define THETA_ENDSTOP_PIN 2   // Theta axis limit switch (X-MAX on RAMPS)

// Endstop pullup configuration (for NO switches wired between pin and GND)
// true = Enable internal pullup (pin HIGH when open, LOW when triggered)
// false = No pullup (requires external pullup/pulldown resistor)
#define RADIUS_ENDSTOP_PULLUP true   // Enable internal pullup for NO switch
#define THETA_ENDSTOP_PULLUP true    // Enable internal pullup for NO switch

#endif // VALUES_H
