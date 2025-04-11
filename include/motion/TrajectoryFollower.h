#pragma once

#include "Trajectory.h"
#include "TrajectoryTypes.h"
#include "utils/MathUtils.h"
#include "constants.h"
#include <chrono>
#include <cmath>

class TrajectoryFollower {
public:
    using Clock = std::chrono::steady_clock;

    TrajectoryFollower(const Motion::Trajectory& trajectory, double toleranceSec = constants::autonomous::TIME_TOLERANCE)
        : trajectory(trajectory), tolerance(toleranceSec) {
        startTime = Clock::now();
    }

    void reset(const Motion::Trajectory& newTrajectory) {
        trajectory = newTrajectory;
        startTime = Clock::now();
        finished = false;
    }

    Motion::TrajectoryPoint update(const Motion::Pose2D& currentPose) {
        if (trajectory.getPoints().empty()) return Motion::TrajectoryPoint{};

        double elapsedSec = std::chrono::duration<double>(Clock::now() - startTime).count();

        auto target = MathUtils::interpolate(elapsedSec, trajectory);

        double lastTime = trajectory.getPoints().back().t;
        if (std::abs(elapsedSec - lastTime) < tolerance) {
            finished = true;
        }

        if (finished) return trajectory.getPoints().back();

        return target;
    }

    bool isFinished() const {
        return finished;
    }

private:
    Motion::Trajectory trajectory;
    Clock::time_point startTime;
    bool finished = false;
    double tolerance;
};
