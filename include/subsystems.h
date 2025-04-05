#ifndef __SUBSYSTEMS_H__
#define __SUBSYSTEMS_H__

#include "subsystems/swerve_drive.h"

inline SwerveDrive *drivebase = nullptr;

// Initialize the subsystems
inline void subsystems_initialize()
{
    drivebase = new SwerveDrive(true);
}

#endif
