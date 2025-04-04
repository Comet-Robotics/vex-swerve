#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "tasks/teleop.h"
#include "subsystems.h"

using namespace pros;

void opcontrol_initialize() {}

static void drivebase_controls(Controller &controller) {
    double forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
    double strafe = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
    double rotation = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

    drivebase->setModuleSpeeds(forward, strafe, rotation);
    drivebase->loop();
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
void opcontrol() {
    Controller controller(pros::E_CONTROLLER_MASTER);

    while (true) {
        pros::lcd::print(0, "Battery: %2.3f V", pros::battery::get_voltage() / 1000.0f);

        drivebase_controls(controller);

        // pros::lcd::print(1, "Rotation Sensor: %i", constants::drivebase::VERTICAL_ROTATION.get_position());

        pros::delay(constants::TELEOP_POLL_TIME);
    }
}