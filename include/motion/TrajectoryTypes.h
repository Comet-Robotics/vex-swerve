// TrajectoryTypes.h
#ifndef TRAJECTORY_TYPES_H
#define TRAJECTORY_TYPES_H

namespace Motion {

struct Pose2D {
    double x;
    double y;
    double heading;
};

struct TrajectoryPoint {
    Pose2D pose;
    Pose2D velocity;
    double t;
};

} // namespace Trajectory

#endif
