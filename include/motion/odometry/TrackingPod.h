#pragma once

#include <cstdint>
#include <math.h>
#include "pros/rotation.hpp"
#include "utils/AngleUtils.h"

class TrackingPod {
public:
    TrackingPod(int8_t sensorPort, double wheelDiameter, double offset) :
    sensor(sensorPort),
    wheelRadius(wheelDiameter/2),
    offset(offset) {}

    double getRawAngle() const {
        return sensor.get_position()/100.0;
    }

    double getOffset() const {
        return offset;
    }

    void tare() {
        sensor.reset_position();
    }

    double getTraveledDistance() const {
        return AngleUtils::toRadians(getRawAngle()) * wheelRadius;
    }

private:
    pros::Rotation sensor;
    double wheelRadius, offset;
};