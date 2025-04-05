#include <cstdint>
#include <array>
#include "pros/imu.hpp"
#include "pros/abstract_motor.hpp"

namespace constants {
    namespace ports {

        // for each of these, first num is the motor controlling the top gear, second is controlling the bottom gear
        // 1
        inline constexpr std::array<int8_t, 2> FRONT_RIGHT_PORTS = {
            0,
            0
        };
        // 2
        inline constexpr std::array<int8_t, 2> FRONT_LEFT_PORTS = {
            0,
            0
        };
        // 3
        inline constexpr std::array<int8_t, 2> BACK_LEFT_PORTS = {
            0,
            0
        };
        // 4
        inline constexpr std::array<int8_t, 2> BACK_RIGHT_PORTS = {
            0,
            0
        };
        
        inline constexpr int8_t IMU_PORT = 0;
    }

    namespace drivetrain {
        inline constexpr pros::MotorGears CHASSIS_INTERNAL_GEARSET = pros::MotorGears::blue;

        inline constexpr int WHEEL_DIAMETER = 0;
        
        inline pros::Imu IMU(ports::IMU_PORT);

        inline constexpr double ROTATION_FACTOR = 1;
        inline constexpr double TRACK_LENGTH = 0.0; // distance between front and back wheels
        inline constexpr double TRACK_WIDTH = 0.0; // distance between left and right wheels

        inline constexpr std::array<double, 3> FRONT_LEFT_PID = {
            0.0, 
            0.0, 
            0.0
        };
        inline constexpr std::array<double, 3> FRONT_RIGHT_PID = {
            0.0, 
            0.0, 
            0.0
        };
        inline constexpr std::array<double, 3> BACK_LEFT_PID = {
            0.0, 
            0.0, 
            0.0
        };
        inline constexpr std::array<double, 3> BACK_RIGHT_PID = {
            0.0, 
            0.0, 
            0.0
        };
    }

    constexpr int TELEOP_POLL_TIME = 20; // milliseconds
}