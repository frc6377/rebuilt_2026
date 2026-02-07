package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

/**
 * Shared constants used by both left and right shooters. Per-shooter constants live in left/LeftShooterConstants and
 * right/RightShooterConstants.
 */
public class ShooterConstants {
    // ==================== Feature Flags ====================
    /** Set to false to disable the hood motor entirely (for robots without a hood) */
    public static final boolean hoodEnabled = false;

    /** Fixed hood angle to use when hood is disabled (degrees) */
    public static final Angle fixedHoodAngle = Degrees.of(45.0);

    // ==================== Operational Limits ====================
    public static final AngularVelocity maxFlywheelVelocity = RotationsPerSecond.of(100.0); // 6000 RPM

    // Velocity tolerance
    public static final AngularVelocity flywheelVelocityTolerance = RotationsPerSecond.of(100.0 / 60.0); // 100 RPM
    public static final AngularVelocity maxVelocityDifference =
            RotationsPerSecond.of(500.0 / 60.0); // 500 RPM difference

    // Acceleration mismatch thresholds
    public static final double accelMismatchToleranceRPM = 150.0;
    public static final int accelMismatchCycles = 10;

    // ==================== Simulation Constants ====================
    public static final Distance shooterHeight = Meters.of(0.5);
    public static final Distance shooterOffsetX = Meters.of(0.3);
    public static final Distance shooterOffsetY = Meters.of(0.0);
    public static final Distance flywheelRadius = Inches.of(2);
    public static final double launchEfficiency = 0.85;

    // ==================== Shooting/Trajectory Constants ====================
    public static final Angle minHoodAngle = Degrees.of(25.0);
    public static final Angle maxHoodAngle = Degrees.of(80.0);

    public static final AngularVelocity minShootingFlywheelVelocity = RPM.of(1500.0);
    public static final AngularVelocity maxShootingFlywheelVelocity = RPM.of(6000.0);

    public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);

    // Shot map defaults (tunable via NetworkTables)
    public static final double defaultMaxHeightFeet = 8.0;
    public static final double defaultTargetHeightFeet = 6.0;

    // Fine-tuning defaults
    public static final double defaultHoodAngleOffset = 0.0;
    public static final double defaultRpmMultiplier = 1.0;
}
