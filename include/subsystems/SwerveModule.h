#ifndef __SUBSYSTEMS_SWERVE_MODULE_H__
#define __SUBSYSTEMS_SWERVE_MODULE_H__

#include "constants.h"
#include "api.h"
#include "utils/PID.h"
#include "utils/AngleUtils.h"
#include <cmath>
#include <array>

using namespace constants::drivetrain;

class SwerveModule {
public:
    SwerveModule(int8_t topMotorPort, int8_t bottomMotorPort) :   
    rotationPID(0, 0, 0, 0),
    topMotor(topMotorPort, CHASSIS_INTERNAL_GEARSET),
    bottomMotor(bottomMotorPort, CHASSIS_INTERNAL_GEARSET) {
        topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        bottomMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    double getModuleRotation() const {
        return ROTATION_FACTOR * (topMotor.get_position() - bottomMotor.get_position());
    }

    void setSpeed(double speed) {
        this->speed = speed;
    }

    void setAngle(double angle) {
        this->angle = angle;
    }

    void setSpeedAndAngle(double targetSpeed, double targetAngle) {
        double currentAngle = getModuleRotation();
        AngleUtils::optimizeAngleAndSpeed(currentAngle, targetAngle, targetSpeed);

        setSpeed(targetSpeed);
        setAngle(targetAngle);
    }

    void setPID(const std::array<double, 3>& coefficients) {
        rotationPID.setCoefficients(coefficients[0], coefficients[1], coefficients[2]);
    }

    void setSpeeds(double topSpeed, double bottomSpeed) {
        topMotor.move_voltage(topSpeed * 12000);
        bottomMotor.move_voltage(bottomSpeed * 12000);
    }

    void update() {
        double currentAngle = fmod(getModuleRotation(), 360.0);
        if (currentAngle < 0) currentAngle += 360.0;
        double rotationPower = rotationPID.calculate(currentAngle, angle);

        double topPower = speed + rotationPower;
        double bottomPower = speed - rotationPower;

        double maxPower = fmax(fabs(topPower), fabs(bottomPower));
        if (maxPower > 1) {
            topPower /= maxPower;
            bottomPower /= maxPower;
        }

        topMotor.move_voltage(topPower * 12000);
        bottomMotor.move_voltage(bottomPower * 12000);
    }

private:
    pros::Motor topMotor, bottomMotor;
    bool fieldCentric;
    PID rotationPID;

    double speed = 0;
    double angle = 0;
};

#endif
