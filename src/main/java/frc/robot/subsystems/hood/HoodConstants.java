// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public class HoodConstants {
    // CAN ID
    public static final int motorID = 22; // Kraken x44

    // CAN bus name
    public static final String canBusName = "rio";

    // Feature flags
    public static final boolean enable = true; // Set to false to disable hood motor initialization

    // Gear ratio
    public static final double gearRatio = 100.0; // Adjust based on actual mechanism

    // Default PID constants for position control
    public static final double defaultKP = 10.0;
    public static final double defaultKI = 0.0;
    public static final double defaultKD = 0.5;

    // Current limit
    public static final Current currentLimit = Amps.of(40.0);

    // Operational limits
    public static final Angle minAngle = Degrees.of(20.0);
    public static final Angle maxAngle = Degrees.of(80.0);
}
