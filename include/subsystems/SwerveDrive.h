#pragma once

#include "motion/TrajectoryTypes.h"
#include "utils/AngleUtils.h"

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

        setPose({0, 0, 0});
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

        frontRight->setSpeedAndAngle(hypot(b, c), AngleUtils::toDegrees(atan2(b, c)));
        frontLeft->setSpeedAndAngle(hypot(b, d), AngleUtils::toDegrees(atan2(b, d)));
        backLeft->setSpeedAndAngle(hypot(a, d), AngleUtils::toDegrees(atan2(a, d)));
        backRight->setSpeedAndAngle(hypot(a, c), AngleUtils::toDegrees(atan2(a, c)));
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

    void setPose(const Motion::Pose2D& pose) {
        currentPose = pose;
    }

    Motion::Pose2D getPose() {
        return currentPose;
    }

    void tareIMU() {
        constants::drivetrain::IMU.tare();
    }

    void setTrajectory(const Motion::Trajectory& trajectory) {
        follower.reset(trajectory);
    }

    void setTrajectory(const std::string& filename) {
        try {
            Motion::Trajectory trajectory = Motion::Trajectory::fromFile(filename);
            setTrajectory(trajectory);
        } catch (const std::exception& e) {
            printf("Failed to load trajectory: %s\n", e.what());
        }
    }

    bool atEnd() const {
        return follower.isFinished();
    }

private:
    bool autonomous = false;

    TrajectoryFollower follower{Motion::Trajectory(), constants::autonomous::TIME_TOLERANCE};
    HolonomicController controller{X_PID, Y_PID, THETA_PID};

    Motion::Pose2D currentPose;
};