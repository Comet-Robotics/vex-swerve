#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "tasks/teleop.h"
#include "subsystems.h"

using namespace pros;

Controller controller(pros::E_CONTROLLER_MASTER);

enum class OpcontrolMode {
    TEST,
    COMP
};

inline constexpr OpcontrolMode MODE = OpcontrolMode::TEST;

void opcontrol_initialize() {}

static void drivebase_controls() {
    double forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double strafe = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    double rotation = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        drivebase->tareIMU();
    }

    drivebase->setModuleSpeeds(forward, strafe, rotation);
    drivebase->update();
}

static void testPodsDrive() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        drivebase->frontRight->setSpeeds(1, 1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        drivebase->frontRight->setSpeeds(-1, -1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        drivebase->frontLeft->setSpeeds(1, 1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        drivebase->frontLeft->setSpeeds(-1, -1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        drivebase->backLeft->setSpeeds(1, 1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        drivebase->backLeft->setSpeeds(-1, -1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        drivebase->backRight->setSpeeds(1, 1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        drivebase->backRight->setSpeeds(-1, -1);
    } else {
        drivebase->frontRight->setSpeeds(0, 0);
        drivebase->frontLeft->setSpeeds(0, 0);
        drivebase->backRight->setSpeeds(0, 0);
        drivebase->backLeft->setSpeeds(0, 0);
    }
}

static void testPodsRotation() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        drivebase->frontRight->setSpeeds(1, -1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        drivebase->frontRight->setSpeeds(-1, 1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        drivebase->frontLeft->setSpeeds(1, -1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        drivebase->frontLeft->setSpeeds(-1, 1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        drivebase->backLeft->setSpeeds(1, -1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        drivebase->backLeft->setSpeeds(-1, 1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        drivebase->backRight->setSpeeds(1, -1);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        drivebase->backRight->setSpeeds(-1, 1);
    } else {
        drivebase->frontRight->setSpeeds(0, 0);
        drivebase->frontLeft->setSpeeds(0, 0);
        drivebase->backRight->setSpeeds(0, 0);
        drivebase->backLeft->setSpeeds(0, 0);
    }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrolTest() {
    while (true) {
        pros::lcd::print(0, "Battery: %2.3f V", pros::battery::get_voltage() / 1000.0f);

        // drivebase_controls(controller);

        static bool testPodsMode = true; // true for drive, false for rotation

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            testPodsMode = !testPodsMode;
        }

        if (testPodsMode) {
            testPodsDrive();
        } else {
            testPodsRotation();
        }

        // pros::lcd::print(1, "Rotation Sensor: %i", constants::drivebase::VERTICAL_ROTATION.get_position());

        pros::delay(constants::TELEOP_POLL_TIME);
    }
}

void opcontrolComp() {
    while (true) {
        drivebase_controls();
        pros::delay(constants::TELEOP_POLL_TIME);
    }
}

void opcontrol() {
    drivebase->setAutonomous(false);

    switch (MODE) {
        case OpcontrolMode::TEST:
            return opcontrolTest();
        case OpcontrolMode::COMP:
            return opcontrolComp();
        default:
            break;
    }
}