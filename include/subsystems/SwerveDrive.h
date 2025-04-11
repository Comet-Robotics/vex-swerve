#ifndef __SUBSYSTEMS_SWERVE_DRIVE_H__
#define __SUBSYSTEMS_SWERVE_DRIVE_H__

#include "utils/AngleUtils.h"
#define _USE_MATH_DEFINES

#include "SwerveModule.h"
#include <cmath>
#include "motion/HolonomicController.h"
#include "motion/TrajectoryFollower.h"

using namespace constants::drivetrain;

class SwerveDrive {
public:
    std::unique_ptr<SwerveModule> frontRight, frontLeft, backLeft, backRight;

    SwerveDrive() : 
    frontRight(std::make_unique<SwerveModule>(constants::ports::FRONT_RIGHT_PORTS[0], constants::ports::FRONT_RIGHT_PORTS[1])),
    frontLeft(std::make_unique<SwerveModule>(constants::ports::FRONT_LEFT_PORTS[0], constants::ports::FRONT_LEFT_PORTS[1])),
    backLeft(std::make_unique<SwerveModule>(constants::ports::BACK_LEFT_PORTS[0], constants::ports::BACK_LEFT_PORTS[1])),
    backRight(std::make_unique<SwerveModule>(constants::ports::BACK_RIGHT_PORTS[0], constants::ports::BACK_RIGHT_PORTS[1])) {
        frontRight->setPID(constants::drivetrain::FRONT_RIGHT_PID);
        frontLeft->setPID(constants::drivetrain::FRONT_LEFT_PID);
        backLeft->setPID(constants::drivetrain::BACK_LEFT_PID);
        backRight->setPID(constants::drivetrain::BACK_RIGHT_PID);

        controller.setCommandCallback([this](double forward, double strafe, double rotation, bool fieldCentric) {
            setModuleSpeeds(forward, strafe, rotation, fieldCentric);
        });
    }

    void setModuleSpeeds(double forward, double strafe, double rotation, bool fieldCentric = true) {
        if (fieldCentric) {
            double angle = AngleUtils::toRadians(constants::drivetrain::IMU.get_rotation());
            double newStrafe = strafe * cos(angle) - forward * sin(angle);
            double newForward = forward * cos(angle) + strafe * sin(angle);
            strafe = newStrafe;
            forward = newForward;
        }

        double a = strafe - rotation;
        double b = strafe + rotation;
        double c = forward - rotation;
        double d = forward + rotation;

        frontRight->setSpeedAndAngle(hypot(b, c), atan2(b, c) * 180.0 / M_PI);
        frontLeft->setSpeedAndAngle(hypot(b, d), atan2(b, d) * 180.0 / M_PI);
        backLeft->setSpeedAndAngle(hypot(a, d), atan2(a, d) * 180.0 / M_PI);
        backRight->setSpeedAndAngle(hypot(a, c), atan2(a, c) * 180.0 / M_PI);
    }

    void update() {
        if (autonomous) {
            Motion::Pose2D currentPose = getPose();
            Motion::TrajectoryPoint targetPoint = follower.update(currentPose);
            auto commands = controller.update(currentPose, targetPoint);
            setModuleSpeeds(std::get<0>(commands), std::get<1>(commands), std::get<2>(commands), true);
        }

        frontRight->update();
        frontLeft->update();
        backLeft->update();
        backRight->update();
    }

    void setAutonomous(bool autonomous) {
        this->autonomous = autonomous;
    }

    Motion::Pose2D getPose() {
        // placeholder for actual pose estimation logic
        return Motion::Pose2D{0, 0, 0};
    }

    void tareIMU() {
        constants::drivetrain::IMU.tare();
    }

private:
    bool autonomous = false;
    TrajectoryFollower follower{Motion::Trajectory(), constants::autonomous::TIME_TOLERANCE};

    HolonomicController controller{X_PID, Y_PID, THETA_PID};
};

#endif
