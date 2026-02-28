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

package frc.robot.subsystems.upgoer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class UpgoerConstants {
    // CAN bus name
    public static final String canBusName = "rio";

    // Feature flags
    public static final boolean enabled = Constants.EnabledSubsystems.kUpgoer;

    // Default PID constants for velocity control
    public static final double defaultKP = 1;
    public static final double defaultKI = 0.0;
    public static final double defaultKD = 0.0;
    public static final double defaultKV = 0.0;
    public static final double defaultKS = 0.0;

    // Current limit
    public static final Current currentLimit = Amps.of(60.0);

    // Default feed velocity
    public static final AngularVelocity defaultFeedVelocity = RPM.of(-4500.0);
    public static final AngularVelocity defaultUnjamVelocity = RPM.of(4500.0);
}
