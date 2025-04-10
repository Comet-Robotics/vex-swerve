#ifndef HOLONOMIC_CONTROLLER_H
#define HOLONOMIC_CONTROLLER_H

#include "utils/AngleUtils.h"
#include "utils/PID.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>

struct Pose2D {
    double x;
    double y;
    double heading;
};

struct TrajectoryPoint {
    Pose2D pose;
    Pose2D velocity;
};

class HolonomicController {
public:
    HolonomicController(std::array<double, 3> xPIDCoefficients, 
                        std::array<double, 3> yPIDCoefficients, 
                        std::array<double, 3> thetaPIDCoefficients)
        : xPID(xPIDCoefficients[0], xPIDCoefficients[1], xPIDCoefficients[2]),
          yPID(yPIDCoefficients[0], yPIDCoefficients[1], yPIDCoefficients[2]),
          thetaPID(thetaPIDCoefficients[0], thetaPIDCoefficients[1], thetaPIDCoefficients[2]) {}

    void setCommandCallback(std::function<void(double forward, double strafe, double omega, bool fieldCentric)> callback) {
        commandCallback = callback;
    }

    void update(const Pose2D& currentPose, const TrajectoryPoint& target) {
        Pose2D targetPose = target.pose;
        Pose2D targetVelocity = target.velocity;

        double dx = targetPose.x - currentPose.x;
        double dy = targetPose.y - currentPose.y;

        double sinHeading = sin(currentPose.heading);
        double cosHeading = cos(currentPose.heading);
        double errorX = dx * cosHeading + dy * sinHeading;
        double errorY = -dx * sinHeading + dy * cosHeading;
        double errorTheta = AngleUtils::shortestAngleDelta(currentPose.heading, targetPose.heading);

        double correctionX = xPID.calculate(errorX);
        double correctionY = yPID.calculate(errorY);
        double correctionTheta = thetaPID.calculate(errorTheta);

        double sinGoal = sin(targetPose.heading);
        double cosGoal = cos(targetPose.heading);
        double vx = targetVelocity.x * cosGoal + targetVelocity.y * sinGoal;
        double vy = -targetVelocity.x * sinGoal + targetVelocity.y * cosGoal;

        double forward = vx + correctionX;
        double strafe = vy + correctionY;
        double rotation = correctionTheta;

        if (commandCallback) {
            commandCallback(forward, strafe, rotation, false);
        }

        logError(currentPose, targetPose);
    }

private:
    PID xPID, yPID, thetaPID;
    std::function<void(double, double, double, bool)> commandCallback;

    void logError(const Pose2D& currentPose, const Pose2D& targetPose) {
        printf("[Tracking] ΔX: %.2f, ΔY: %.2f, Δθ: %.2f\n", 
               targetPose.x - currentPose.x, 
               targetPose.y - currentPose.y, 
               targetPose.heading - currentPose.heading);
    }
};

#endif
