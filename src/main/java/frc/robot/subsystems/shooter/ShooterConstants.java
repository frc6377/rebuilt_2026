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
    // ==================== Feature Flags ====================
    /** Set to false to disable the hood motor entirely (for robots without a hood) */
    public static final boolean hoodEnabled = true;

    /** Set to false to disable left flywheel motor */
    public static final boolean leftFlywheelEnabled = true;

    /** Set to false to disable right flywheel motor */
    public static final boolean rightFlywheelEnabled = true;

    /** Fixed hood angle to use when hood is disabled (degrees) */
    public static final Angle fixedHoodAngle = Degrees.of(45.0);

    // CAN IDs
    public static final int leftFlywheelMotorID = 20; // Kraken X60
    public static final int rightFlywheelMotorID = 21; // Kraken X60
    public static final int leftSpinMotorID = 22; // Kraken X44 - spin adjustment on left hood
    public static final int rightSpinMotorID = 23; // Kraken X44 - spin adjustment on right hood

    // CAN bus name
    public static final String canBusName = "rio";

    // Default PID constants for flywheel (velocity control)
    public static final double defaultFlywheelKP = 0.1;
    public static final double defaultFlywheelKI = 0.0;
    public static final double defaultFlywheelKD = 0.0;
    public static final double defaultFlywheelKV = 0;
    public static final double defaultFlywheelKS = 0.0;

    // Current limits
    public static final Current flywheelCurrentLimit = Amps.of(60.0);
    public static final Current spinMotorCurrentLimit = Amps.of(40.0); // Lower limit for X44

    // Default PID constants for spin motors (velocity control)
    public static final double defaultSpinKP = 0.1;
    public static final double defaultSpinKI = 0.0;
    public static final double defaultSpinKD = 0.0;
    public static final double defaultSpinKV = 0.0;
    public static final double defaultSpinKS = 0.0;

    // Default spin ratio (spin motor velocity as fraction of main flywheel velocity)
    public static final double defaultSpinRatio = 0.5;

    // Operational limits
    public static final AngularVelocity maxFlywheelVelocity = RotationsPerSecond.of(100.0); // 6000 RPM

    // Limp mode thresholds
    public static final AngularVelocity flywheelVelocityTolerance = RotationsPerSecond.of(100.0 / 60.0); // 100 RPM
    public static final AngularVelocity maxVelocityDifference =
            RotationsPerSecond.of(500.0 / 60.0); // 500 RPM difference between flywheels

    // Simulation constants
    public static final Distance shooterHeight = Meters.of(0.5); // Height of shooter from ground
    public static final Distance shooterOffsetX = Meters.of(0.3); // Forward offset from robot center
    public static final Distance shooterOffsetY = Meters.of(0.0); // Lateral offset from robot center
    public static final Distance flywheelRadius = Inches.of(2); // Flywheel wheel radius
    public static final double launchEfficiency = 0.85; // Energy transfer efficiency (0.0 - 1.0)

    // ==================== Shooting/Trajectory Constants ====================

    // Hood angle limits
    public static final Angle minHoodAngle = Degrees.of(25.0);
    public static final Angle maxHoodAngle = Degrees.of(80.0);

    // Flywheel velocity limits for shooting
    public static final AngularVelocity minShootingFlywheelVelocity = RPM.of(1500.0);
    public static final AngularVelocity maxShootingFlywheelVelocity = RPM.of(6000.0);

    // Trajectory physics
    public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(11); // Accounts for drag etc.

    // Trajectory target heights (defaults, tunable via NetworkTables)
    public static final double defaultMaxHeightFeet = 8.0; // Peak height of ball trajectory
    public static final double defaultTargetHeightFeet = 6.0; // Hub opening height

    // Fine-tuning defaults
    public static final double defaultHoodAngleOffset = 0.0; // Degrees offset for hood angle
    public static final double defaultRpmMultiplier = 1.0; // Multiplier for calculated RPM
}
