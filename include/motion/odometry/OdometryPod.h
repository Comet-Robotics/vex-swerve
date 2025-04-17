#pragma once

#include <cstdint>
#include <math.h>
#include "pros/rotation.hpp"
#include "utils/AngleUtils.h"

class OdometryPod {
public:
    OdometryPod(int8_t sensorPort, double wheelDiameter) :
    sensor(sensorPort),
    wheelRadius(wheelDiameter/2) {}

    void tare() {
        sensor.reset_position();
    }

    double getTraveledDistance() const {
        return AngleUtils::toRadians(sensor.get_position()/100.0) * wheelRadius;
    }

private:
    pros::Rotation sensor;
    double wheelRadius;
};