#pragma once

#include "subsystems/SwerveDrive.h"

inline SwerveDrive *drivebase = nullptr;

// Initialize the subsystems
inline void subsystems_initialize()
{
    drivebase = new SwerveDrive();
}