#ifndef __SUBSYSTEMS_SWERVE_DRIVE_H__
#define __SUBSYSTEMS_SWERVE_DRIVE_H__

#define _USE_MATH_DEFINES

#include "swerve_module.h"
#include <cmath>

using namespace constants::drivetrain;

class SwerveDrive {
public:
    SwerveDrive(bool fieldCentric) : 
    frontRight(std::make_unique<SwerveModule>(constants::ports::FRONT_RIGHT_PORTS[0], constants::ports::FRONT_RIGHT_PORTS[1])),
    frontLeft(std::make_unique<SwerveModule>(constants::ports::FRONT_LEFT_PORTS[0], constants::ports::FRONT_LEFT_PORTS[1])),
    backLeft(std::make_unique<SwerveModule>(constants::ports::BACK_LEFT_PORTS[0], constants::ports::BACK_LEFT_PORTS[1])),
    backRight(std::make_unique<SwerveModule>(constants::ports::BACK_RIGHT_PORTS[0], constants::ports::BACK_RIGHT_PORTS[1])),
    fieldCentric(fieldCentric) {
        frontRight->setPID(constants::drivetrain::FRONT_RIGHT_PID);
        frontLeft->setPID(constants::drivetrain::FRONT_LEFT_PID);
        backLeft->setPID(constants::drivetrain::BACK_LEFT_PID);
        backRight->setPID(constants::drivetrain::BACK_RIGHT_PID);
    }

    void setModuleSpeeds(double forward, double strafe, double rotation) {
        if (fieldCentric) {
            double angle = constants::drivetrain::IMU.get_rotation() * M_PI / 180.0;
            double newStrafe = strafe * cos(angle) - forward * sin(angle);
            double newForward = forward * cos(angle) + strafe * sin(angle);
            strafe = newStrafe;
            forward = newForward;
        }

        double a = strafe - rotation * TRACK_LENGTH / 2.0;
        double b = strafe + rotation * TRACK_LENGTH / 2.0;
        double c = forward - rotation * TRACK_WIDTH / 2.0;
        double d = forward + rotation * TRACK_WIDTH / 2.0;

        frontRight->setSpeedAndAngle(hypot(b, c), atan2(b, c) * 180.0 / M_PI);
        frontLeft->setSpeedAndAngle(hypot(b, d), atan2(b, d) * 180.0 / M_PI);
        backLeft->setSpeedAndAngle(hypot(a, d), atan2(a, d) * 180.0 / M_PI);
        backRight->setSpeedAndAngle(hypot(a, c), atan2(a, c) * 180.0 / M_PI);
    }

    void loop() {
        frontRight->loop();
        frontLeft->loop();
        backLeft->loop();
        backRight->loop();
    }

private:
    std::unique_ptr<SwerveModule> frontRight, frontLeft, backLeft, backRight;
    bool fieldCentric;
};

#endif
