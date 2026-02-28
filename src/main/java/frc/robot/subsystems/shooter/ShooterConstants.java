package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import java.util.HashMap;

/**
 * Shared constants used by both left and right shooters. Per-shooter constants live in left/LeftShooterConstants and
 * right/RightShooterConstants.
 */
public class ShooterConstants {
    // ==================== Feature Flags ====================
    /** Set to false to disable the hood motor entirely (for robots without a hood) */
    public static final boolean kHoodEnabled = Constants.EnabledSubsystems.kHood;

    /** Set to true to enable Shooting on the Fly compensation */
    public static final boolean kSotfEnabled = true;

    public enum CalculationMode {
        PHYSICS,
        DOU_INTERPOLATION
    }

    public static final CalculationMode kDefaultCalculationMode = CalculationMode.DOU_INTERPOLATION;
    public static final boolean kManualShootingEnabled = true;
    public static final int kManualShootingSpeedRPM = 3600;

    /** Fixed hood angle to use when hood is disabled (degrees) */
    public static final Angle kFixedHoodAngle = Degrees.of(60);

    // ==================== Operational Limits ====================
    public static final AngularVelocity kMaxFlywheelVelocity = RotationsPerSecond.of(100.0); // 6000 RPM

    // Velocity tolerance
    public static final AngularVelocity kFlywheelVelocityTolerance = RotationsPerSecond.of(2); // 120 RPM
    public static final AngularVelocity kMaxVelocityDifference =
            RotationsPerSecond.of(500.0 / 60.0); // 500 RPM difference

    // Acceleration mismatch thresholds
    public static final double kAccelMismatchToleranceRPM = 1000000;
    public static final int kAccelMismatchCycles = 100000000;

    // ==================== Simulation Constants ====================
    public static final Distance shooterHeight = Meters.of(0.5);
    public static final Distance shooterOffsetXLeft = Meters.of(0.3);
    public static final Distance shooterOffsetYLeft = Meters.of(0.0);
    public static final Distance shooterOffsetXRight = Meters.of(0.3);
    public static final Distance shooterOffsetYRight = Meters.of(0.0);
    public static final Distance flywheelRadius = Inches.of(2);
    public static final double launchEfficiency = 0.75; // Percentage of theoretical velocity achieved at the target

    // ==================== Shooting/Trajectory Constants ====================
    public static final Angle minHoodAngle = Degrees.of(25.0);
    public static final Angle maxHoodAngle = Degrees.of(80.0);

    public static final AngularVelocity minShootingFlywheelVelocity = RPM.of(1500.0);
    public static final AngularVelocity maxShootingFlywheelVelocity = RPM.of(6000.0);

    public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(11);
    public static final AngularVelocity kDefaultUnjamVelocity = RPM.of(-1500);
    // Shot map defaults (tunable via NetworkTables)
    public static final double defaultMaxHeightFeet = 8.0;
    public static final double defaultTargetHeightFeet = 6.0;

    // Fine-tuning defaults
    public static final double defaultHoodAngleOffset = 0.0;
    public static final double defaultRpmMultiplier = 1.3;

    // Bench Mode Defaults
    public static final double defaultBenchModeEnabled = 1;
    public static final double defaultBenchModeDistanceMeters = 10.0;
    // Shooter Tuning
    public static final HashMap<Distance, AngularVelocity> distanceToAngularVelocity = new HashMap<>();

    static {
        distanceToAngularVelocity.put(Meters.of(2.0), RPM.of(2750));
        distanceToAngularVelocity.put(Meters.of(3.0), RPM.of(3156));
        distanceToAngularVelocity.put(Meters.of(4.0), RPM.of(3512));
        distanceToAngularVelocity.put(Meters.of(5.0), RPM.of(3823));
        distanceToAngularVelocity.put(Meters.of(6.0), RPM.of(4095));
    }

    public static final InterpolatingDoubleTreeMap distanceToAngularVelocityDouMapRPM =
            new InterpolatingDoubleTreeMap();

    static {
        for (Distance distance : distanceToAngularVelocity.keySet()) {
            distanceToAngularVelocityDouMapRPM.put(
                    distance.in(Meters), distanceToAngularVelocity.get(distance).in(RPM));
        }
    }
}
