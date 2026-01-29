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

public class ShooterConstants {
    // CAN IDs
    public static final int leftFlywheelMotorID = 20; // Kraken X60
    public static final int rightFlywheelMotorID = 21; // Kraken X60
    public static final int hoodMotorID = 22; // Kraken x44

    // CAN bus name
    public static final String canBusName = "rio";

    // Flywheel PID constants (velocity control)
    public static final double flywheelKP = 0.1;
    public static final double flywheelKI = 0.0;
    public static final double flywheelKD = 0.0;
    public static final double flywheelKV = 0.12;
    public static final double flywheelKS = 0.0;

    // Hood PID constants (position control)
    public static final double hoodKP = 10.0;
    public static final double hoodKI = 0.0;
    public static final double hoodKD = 0.5;

    // Gear ratios
    public static final double hoodGearRatio = 100.0; // Adjust based on actual mechanism

    // Current limits (Amps)
    public static final double flywheelCurrentLimit = 60.0;
    public static final double hoodCurrentLimit = 40.0;

    // Operational limits
    public static final double maxFlywheelRPM = 6000.0;
    public static final double minHoodAngleDegrees = 0.0;
    public static final double maxHoodAngleDegrees = 45.0;

    // Limp mode thresholds
    public static final double flywheelVelocityTolerance = 100.0; // RPM
    public static final double maxVelocityDifference = 500.0; // Max RPM difference between flywheels
}
