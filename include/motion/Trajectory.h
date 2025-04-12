#pragma once

#include <stdexcept>
#include <string>
#include "types/TrajectoryPoint.h"
#include "types/Pose2D.h"
#include "simdjson/simdjson.h"

class Trajectory {
public:

    Trajectory() = default;

    static Trajectory fromFile(const std::string& filename) {
        simdjson::ondemand::parser parser;
        simdjson::padded_string json_str = simdjson::padded_string::load(filename);
        simdjson::ondemand::document doc = parser.iterate(json_str);

        auto samples = doc["trajectory"]["samples"];

        if (samples.error() != simdjson::SUCCESS) {
            throw std::runtime_error("Failed to parse trajectory file: " + filename);
        }

        Trajectory trajectory;

        for (auto sample : samples) {
            TrajectoryPoint point;
            
            point.t = double(sample["t"]);

            point.pose.x = double(sample["x"]);
            point.pose.y = double(sample["y"]);
            point.pose.heading = double(sample["heading"]);

            point.velocity.x = double(sample["vx"]);
            point.velocity.y = double(sample["vy"]);
            point.velocity.heading = double(sample["omega"]);

            trajectory.addPoint(point);
        }

        return trajectory;
    }

    void addPoint(TrajectoryPoint point) {
        points.emplace_back(point);
    }

    const std::vector<TrajectoryPoint>& getPoints() const {
        return points;
    }

    void clear() {
        points.clear();
    }

private:
    std::vector<TrajectoryPoint> points;
};