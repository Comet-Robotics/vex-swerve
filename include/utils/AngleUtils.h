#ifndef ANGLE_UTILS_H
#define ANGLE_UTILS_H

#include <cmath>

namespace AngleUtils {

    // Wraps an angle to the range [0, 360)
    inline double wrap360(double angle) {
        angle = fmod(angle, 360.0);
        if (angle < 0) angle += 360.0;
        return angle;
    }

    // Wraps an angle to the range (-180, 180]
    inline double wrap180(double angle) {
        angle = fmod(angle + 180.0, 360.0);
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    inline double shortestAngleDelta(double from, double to) {
        double delta = to - from;
        return wrap180(delta);
    }

    // Returns optimized angle and speed based on delta (minimize rotation)
    inline void optimizeAngleAndSpeed(double currentAngle, double& targetAngle, double& targetSpeed) {
        currentAngle = wrap360(currentAngle);
        targetAngle = wrap360(targetAngle);

        double delta = shortestAngleDelta(currentAngle, targetAngle);

        if (std::abs(delta) > 90.0) {
            targetSpeed *= -1;
            targetAngle = wrap360(targetAngle + 180.0);
        }
    }

}

#endif
