#ifndef __SUBSYSTEMS_SWERVE_MODULE_H__
#define __SUBSYSTEMS_SWERVE_MODULE_H__

#include "constants.h"
#include "pros/adi.hpp"
#include "swerve_module.h"
#include <cmath>

using namespace constants::drivetrain;
class SwerveModule
{

public:
    SwerveModule(int8_t topMotorPort, int8_t bottomMotorPort)
        : topMotor(topMotorPort, CHASSIS_INTERNAL_GEARSET),
          bottomMotor(bottomMotorPort, CHASSIS_INTERNAL_GEARSET)
    {
        topMotor = pros::Motor(topMotorPort, CHASSIS_INTERNAL_GEARSET);
        bottomMotor = pros::Motor(bottomMotorPort, CHASSIS_INTERNAL_GEARSET);
        topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        bottomMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
    
    void setModuleSpeeds(double translationX, double translationY, double rotation)
    {
        double a = translationX - rotation * TRACK_LENGTH / 2;
        double b = translationX + rotation * TRACK_LENGTH / 2;
        double c = translationY - rotation * TRACK_WIDTH / 2;
        double d = translationY + rotation * TRACK_WIDTH / 2;

        frontRight->setSpeedAndAngle(sqrt(b*b + c*c), atan2(b, c) * 180 / pi);
        frontLeft->setSpeedAndAngle(sqrt(b*b + d*d), atan2(b, d) * 180 / pi);
        backLeft->setSpeedAndAngle(sqrt(a*a + d*d), atan2(a, d) * 180 / pi);
        backRight->setSpeedAndAngle(sqrt(a*a + c*c), atan2(a, c) * 180 / pi);

    }

    void setConstants()
    {
        frontRight->setPID()
    }

    void loop()
    {
        frontRight->loop();
        frontLeft->loop();
        backLeft->loop();
        backRight->loop();
    }

private:
    SwerveModule frontLeft, frontRight, backLeft, backRight;
};

#endif