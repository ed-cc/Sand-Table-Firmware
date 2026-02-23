// theta_compensation.cpp - Polar move decomposition with theta-rotation compensation

#include "theta_compensation.h"
#include "values.h"
#include <math.h>

MoveSteps ThetaCompensation::decompose(float deltaRadiusMM, float deltaThetaDeg) {
    MoveSteps result;

    // Theta axis: convert degrees to motor steps
    result.thetaSteps = (int32_t)roundf(deltaThetaDeg * THETA_STEPS_PER_DEGREE);

    // Radius axis: commanded linear movement + theta compensation
    // The compensation keeps the 100T centre pulley synchronised with the
    // theta output during rotation. Positive theta -> positive compensation.
    float radiusCommand = deltaRadiusMM * RADIUS_STEPS_PER_MM;
    float compensation  = deltaThetaDeg * RADIUS_CENTRE_STEPS_PER_DEGREE;
    result.radiusSteps  = (int32_t)roundf(radiusCommand + compensation);

    return result;
}
