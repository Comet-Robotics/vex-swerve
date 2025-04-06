#ifndef HOLONOMIC_CONTROLLER_H
#define HOLONOMIC_CONTROLLER_H

#include <cmath>
#include <iostream>
#include <functional>

// --- Simple Pose structure ---
struct Pose2D {
    double x;
    double y;
    double heading; // in radians
};

// --- Trajectory point structure ---
struct TrajectoryPoint {
    Pose2D pose;
    Pose2D velocity;
};

class HolonomicController {
public:
    HolonomicController(double kP_xy, double kP_theta)
        : kP_xy(kP_xy), kP_theta(kP_theta) {}

    // External callback to command the drivetrain
    void setCommandCallback(std::function<void(double forward, double strafe, double omega)> callback) {
        commandCallback = callback;
    }

    // Call this each loop with the current pose and current point on the trajectory
    void update(const Pose2D& currentPose, const TrajectoryPoint& target) {
        Pose2D targetPose = target.pose;
        Pose2D targetVelocity = target.velocity;

        double dx = targetPose.x - currentPose.x;
        double dy = targetPose.y - currentPose.y;

        double sinHeading = sin(-currentPose.heading);
        double cosHeading = cos(-currentPose.heading);
        double errorX = dx * cosHeading - dy * sinHeading;
        double errorY = dx * sinHeading + dy * cosHeading;

        double headingError = target.pose.heading - currentPose.heading;
        headingError = atan2(sin(headingError), cos(headingError));

        double correctionX = kP_xy * errorX;
        double correctionY = kP_xy * errorY;
        double correctionTheta = kP_theta * headingError;

        double sinGoal = sin(-targetPose.heading);
        double cosGoal = cos(-targetPose.heading);
        double vx = targetVelocity.x * cosGoal - targetVelocity.y * sinGoal;
        double vy = targetVelocity.x * sinGoal + targetVelocity.y * cosGoal;

        double forward = vx + correctionX;
        double strafe = vy + correctionY;
        double rotation = correctionTheta;

        if (commandCallback) {
            commandCallback(forward, strafe, rotation);
        }

        logError(currentPose, targetPose);
    }

private:
    double kP_xy;
    double kP_theta;
    std::function<void(double, double, double)> commandCallback;

    void logError(const Pose2D& currentPose, const Pose2D& targetPose) {
        std::cout << "[Tracking] ΔX: " << (targetPose.x - currentPose.x)
                  << ", ΔY: " << (targetPose.y - currentPose.y)
                  << ", Δθ: " << (targetPose.heading - currentPose.heading) << std::endl;
    }
};

#endif
