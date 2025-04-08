#include "subsystems.h"
#include "subsystems/SwerveModule.h"
#include "tasks/auton.h"

using namespace pros;

enum class AutonMode
{
    TEST
};

/*
 *  VS is 2v2 auton
 *  SKILLS is self explanitory
 *  TEST is testing any autons or tuning
 */
inline constexpr AutonMode MODE = AutonMode::TEST;

void autonomousTest()
{
    while (true) {
        drivebase->setModuleSpeeds(0.5, 0, 0);
        drivebase->update();

        pros::delay(constants::TELEOP_POLL_TIME);
    }
}

void autonomous()
{
    drivebase->setAutonomous(true);

    switch (MODE)
    {
        case AutonMode::TEST:
            return autonomousTest();
        default:
            break;
    }
};

void autonomous_initialize()
{
    
};