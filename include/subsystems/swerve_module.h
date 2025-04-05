#ifndef __SUBSYSTEMS_SWERVE_MODULE_H__
#define __SUBSYSTEMS_SWERVE_MODULE_H__

#include "constants.h"
#include "api.h"
#include "utils/PID.h"
#include <cmath>

using namespace constants::drivetrain;
class SwerveModule
{

public:
    SwerveModule(int8_t topMotorPort, int8_t bottomMotorPort)
        : rotationPID(int8_t kp, int8_t ki, int8_t kd, 0),
            topMotor(topMotorPort, CHASSIS_INTERNAL_GEARSET),
            bottomMotor(bottomMotorPort, CHASSIS_INTERNAL_GEARSET)
    {
        topMotor = pros::Motor(topMotorPort, CHASSIS_INTERNAL_GEARSET);
        bottomMotor = pros::Motor(bottomMotorPort, CHASSIS_INTERNAL_GEARSET);
        topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        bottomMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
    
    double getModuleRotation()
    {
        return ROTATION_FACTOR * (topMotor.get_position() + bottomMotor.get_position())
    }

    void setSpeed(double speed)
    {
        this->speed = speed;
    }

    void setAngle(double angle)
    {
        this->angle = angle;
    }

    void setSpeedAndAngle(double speed, double angle)
    {
        setSpeed(speed);
        setAngle(angle);
    }

    void setPID(const std::array<double, 3>& coefficients)
    {
        rotationPID = PID(coefficients[0], coefficients[1], coefficients[2], 0);
    }

    void loop()
    {
        double currentAngle = getModuleRotation();

        double rotationPower = rotationPID.calculate(currentAngle, angle);

        double topPower = speed + rotationPower;
        double bottomPower = speed - rotationPower;

        double maxPower = abs(max(topPower, bottomPower));
        if (maxPower > 1)
        {
            topPower /= maxPower;
            bottomPower /= maxPower;
        }

        topMotor.move_voltage(topPower * 12000);
        bottomMotor.move_voltage(bottomPower * 12000);
    }

private:
    pros::Motor topMotor, bottomMotor;
    PID rotationPID;

    int speed = 0, angle = 0;
};

#endif