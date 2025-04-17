#pragma once

#include "OdometryPod.h"
#include "pros/imu.hpp"
#include "types/Pose2D.h"
#include "utils/AngleUtils.h"
#include <cstdint>

//TODO: solve the offset problem for the odom pods
class Odometry {
public:
    Odometry(OdometryPod verticalPod, OdometryPod horizontalPod, int8_t imuPort, double wheelDiameter) :
    verticalPod(verticalPod),
    horizontalPod(horizontalPod),
    imu(imuPort) {
        verticalPod.tare();
        horizontalPod.tare();
        imu.set_heading(0);
    }

    void tare(bool calibrate = false) {
        verticalPod.tare();
        horizontalPod.tare();
        calibrate ? imu.reset() : imu.set_heading(0); // idk if .reset() sets heading to 0
    }

    void setPose(double x, double y, double theta) {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    Pose2D getPose(bool degrees = true) const {
        return Pose2D{x,
            y,
            getTheta(degrees)
        };
    }

    double getX() const { return x; }
    double getY() const { return y; }
    double getTheta(bool degrees = false) const { return degrees ? AngleUtils::toDegrees(theta) : theta; }

    void update() {
        double deltaX = horizontalPod.getTraveledDistance() - lastHDist;
        double deltaY = verticalPod.getTraveledDistance() - lastVDist;

        x += deltaX * cos(theta) - deltaY * sin(theta);
        y += deltaX * sin(theta) + deltaY * cos(theta);
        theta = AngleUtils::toRadians(AngleUtils::wrap360(imu.get_rotation()));
        lastHDist = horizontalPod.getTraveledDistance();
        lastVDist = verticalPod.getTraveledDistance();
    }

private:
    OdometryPod verticalPod;
    OdometryPod horizontalPod;
    pros::Imu imu;
    double x = 0.0, y = 0.0, theta = 0.0;
    double lastHDist = 0.0, lastVDist = 0.0;
};