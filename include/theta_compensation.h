// theta_compensation.h - Polar move decomposition with theta-rotation compensation
//
// Converts a polar move (delta_r_mm, delta_theta_deg) into raw axis step counts.
// The radius step count includes compensation steps that keep the 100T centre
// pulley in sync with the rotating theta arm, preventing belt wind-up.

#ifndef THETA_COMPENSATION_H
#define THETA_COMPENSATION_H

#include <stdint.h>

struct MoveSteps {
    int32_t thetaSteps;   // theta motor steps (signed)
    int32_t radiusSteps;  // radius motor steps including compensation (signed)
};

class ThetaCompensation {
public:
    // Decompose a polar move into raw axis step counts.
    // deltaRadiusMM:  commanded radius change (mm, positive = outward)
    // deltaThetaDeg:  commanded theta change (degrees, positive = CCW)
    //
    // Returns signed step counts for both axes. The radius step count includes
    // the theta compensation: radiusSteps = radius_command + compensation.
    // Compensation is positive when theta is positive (same direction).
    static MoveSteps decompose(float deltaRadiusMM, float deltaThetaDeg);
};

#endif // THETA_COMPENSATION_H
