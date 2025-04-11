#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "motion/TrajectoryTypes.h"
#include "motion/Trajectory.h"
#include "AngleUtils.h"
#include <vector>
#include <cmath>

namespace MathUtils {

    template <typename... Args>
    void scaleToUnitRange(Args&... args) {
        std::vector<double*> values = { &args... };
        double maxAbsValue = 0.0;

        // Find the maximum absolute value
        for (double* value : values) {
            maxAbsValue = fmax(maxAbsValue, fabs(*value));
        }

        // Scale values to the range [-1, 1] if maxAbsValue > 1.0
        if (maxAbsValue > 1.0) {
            for (double* value : values) {
                *value /= maxAbsValue;
            }
        }
    }

    inline Motion::TrajectoryPoint interpolate(double t, Motion::Trajectory& trajectory) {
        const auto& points = trajectory.getPoints();
        if (points.empty()) return Motion::TrajectoryPoint{};

        if (t <= points.front().t) return points.front();
        if (t >= points.back().t) return points.back();

        for (size_t i = 1; i < points.size(); ++i) {
            const auto& prev = points[i - 1];
            const auto& next = points[i];
            if (t < next.t) {
                double ratio = (t - prev.t) / (next.t - prev.t);
                return Motion::TrajectoryPoint{
                    .pose = {
                        std::lerp(prev.pose.x, next.pose.x, ratio),
                        std::lerp(prev.pose.y, next.pose.y, ratio),
                        AngleUtils::lerpAngle(prev.pose.heading, next.pose.heading, ratio)
                    },
                    .velocity = {
                        std::lerp(prev.velocity.x, next.velocity.x, ratio),
                        std::lerp(prev.velocity.y, next.velocity.y, ratio),
                        std::lerp(prev.velocity.heading, next.velocity.heading, ratio)
                    },
                    .t = t,
                };
            }
        }

        return points.back();
    }
}

#endif // MATH_UTILS_H