#pragma once

#include "TrackingPod.h"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "types/Pose2D.h"
#include "utils/AngleUtils.h"
#include <cmath>

class Odometry {
public:
    Odometry(TrackingPod* verticalPod,
             TrackingPod* horizontalPod,
             pros::Imu* imu) :
    verticalPod(verticalPod),
    horizontalPod(horizontalPod),
    imu(imu) {}

    void tare(bool calibrate = false) {
        verticalPod->tare();
        horizontalPod->tare();
        if (calibrate) {
            imu->reset();
            while(imu->is_calibrating()) pros::delay(10);
        }
        imu->set_heading(0);
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        lastHDist = 0.0;
        lastVDist = 0.0;
        lastTheta = 0.0;
    }

    void setPose(double x, double y, double theta) {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    inline Pose2D getPose(bool degrees = true) const {
        return Pose2D{x,
            y,
            getTheta(degrees)
        };
    }

    inline double getX() const { return x; }
    inline double getY() const { return y; }
    inline double getTheta(bool degrees = false) const { return degrees ? AngleUtils::toDegrees(theta) : theta; }

    void update() {
        const double currentHDist = horizontalPod->getTraveledDistance();
        const double currentVDist = verticalPod->getTraveledDistance();
        const double currentTheta = AngleUtils::toRadians(imu->get_rotation());

        const double dXRaw = currentHDist - lastHDist;
        const double dYRaw = currentVDist - lastVDist;
        const double dTheta = currentTheta - lastTheta;

        const double dX = dXRaw + horizontalPod->getOffset() * dTheta;
        const double dY = dYRaw - verticalPod->getOffset() * dTheta;

        const double dX_r =  dX * cos(dTheta/2) - dY * sin(dTheta/2);
        const double dY_r =  dX * sin(dTheta/2) + dY * cos(dTheta/2);

        x += dX_r * cos(theta) - dY_r * sin(theta);
        y += dX_r * sin(theta) + dY_r * cos(theta);
        theta = AngleUtils::wrap2Pi(currentTheta);

        lastHDist = horizontalPod->getTraveledDistance();
        lastVDist = verticalPod->getTraveledDistance();
        lastTheta = AngleUtils::toRadians(imu->get_rotation());
    }

private:
    TrackingPod* verticalPod;
    TrackingPod* horizontalPod;
    pros::Imu* imu;
    double x = 0.0, y = 0.0, theta = 0.0;
    double lastHDist = 0.0, lastVDist = 0.0, lastTheta = 0.0;
};