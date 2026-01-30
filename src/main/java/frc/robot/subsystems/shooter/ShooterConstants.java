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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public class ShooterConstants {
    // CAN IDs
    public static final int leftFlywheelMotorID = 20; // Kraken X60
    public static final int rightFlywheelMotorID = 21; // Kraken X60

    // CAN bus name
    public static final String canBusName = "rio";

    // Default PID constants for flywheel (velocity control)
    public static final double defaultFlywheelKP = 0.1;
    public static final double defaultFlywheelKI = 0.0;
    public static final double defaultFlywheelKD = 0.0;
    public static final double defaultFlywheelKV = 0.12;
    public static final double defaultFlywheelKS = 0.0;

    // Current limits
    public static final Current flywheelCurrentLimit = Amps.of(60.0);

    // Operational limits
    public static final AngularVelocity maxFlywheelVelocity = RotationsPerSecond.of(100.0); // 6000 RPM

    // Limp mode thresholds
    public static final AngularVelocity flywheelVelocityTolerance =
            RotationsPerSecond.of(100.0 / 60.0); // 100 RPM
    public static final AngularVelocity maxVelocityDifference =
            RotationsPerSecond.of(500.0 / 60.0); // 500 RPM difference between flywheels
}
