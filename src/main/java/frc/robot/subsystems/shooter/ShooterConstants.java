package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
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
    /** Set to true to enable Shooting on the Fly compensation */
    public static final boolean kSotfEnabled = false;

    public enum CalculationMode {
        PHYSICS,
        DOU_INTERPOLATION
    }

    public static final CalculationMode kDefaultCalculationMode = CalculationMode.DOU_INTERPOLATION;
    public static final boolean kManualShootingEnabled = false;
    public static final int kManualShootingSpeedRPM = 3600;

    /** Fixed hood angle to use (degrees) */
    public static final Angle kFixedHoodAngle = Degrees.of(60);

    // ==================== Operational Limits ====================
    public static final AngularVelocity kMaxFlywheelVelocity = RotationsPerSecond.of(100.0); // 6000 RPM

    // Velocity tolerance
    public static final AngularVelocity kFlywheelVelocityTolerance = RPM.of(150.0);
    public static final Angle kHeadingTolerance = Degrees.of(5);
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
    public static final AngularVelocity minShootingFlywheelVelocity = RPM.of(1500.0);
    public static final AngularVelocity maxShootingFlywheelVelocity = RPM.of(6000.0);

    public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(11);
    public static final AngularVelocity kDefaultUnjamVelocity = RPM.of(-1500);
    // Shot map defaults (tunable via NetworkTables)
    public static final double defaultMaxHeightFeet = 8.0;
    public static final double defaultTargetHeightFeet = 6.0;

    // Fine-tuning defaults
    public static final double defaultRpmMultiplier = 1;

    // Bench Mode Defaults
    public static final double defaultBenchModeEnabled = 0;
    public static final double defaultBenchModeDistanceMeters = 10.0;
    public static final double offsetM = 0.5969;
    // Shooter Tuning
    public static final HashMap<Distance, AngularVelocity> distanceToAngularVelocity = new HashMap<>();

    static {
        distanceToAngularVelocity.put(Meters.of(0.0), RPM.of(1500));
        distanceToAngularVelocity.put(Meters.of(3.18), RPM.of(3600));
        distanceToAngularVelocity.put(Meters.of(1), RPM.of(2400));
        distanceToAngularVelocity.put(Meters.of(6), RPM.of(4000));
    }

    public static final InterpolatingDoubleTreeMap distanceToAngularVelocityDouMapRPM =
            new InterpolatingDoubleTreeMap();

    static {
        for (Distance distance : distanceToAngularVelocity.keySet()) {
            distanceToAngularVelocityDouMapRPM.put(
                    distance.in(Meters) + offsetM,
                    distanceToAngularVelocity.get(distance).in(RPM));
        }
    }

    public record ShooterConfig(
            String name,
            int flywheelLeaderId,
            int flywheelFollowerId,
            String canBusName,
            boolean enabled,
            boolean followerEnabled,
            InvertedValue flywheelInverted,
            // Ramp rates
            Time flywheelOpenLoopRamp,
            Time flywheelClosedLoopRamp,
            // Flywheel PID
            double flywheelKP,
            double flywheelKI,
            double flywheelKD,
            double flywheelKV,
            double flywheelKS,
            double flywheelKA,
            // Current limits
            Current flywheelCurrentLimitStator,
            Current flywheelCurrentLimitSupply,
            boolean flywheelCurrentLimitStatorEnable,
            boolean flywheelCurrentLimitSupplyEnable) {}

    public static final ShooterConfig leftConfig = new ShooterConfig(
            "LeftShooter",
            Constants.CANIDs.MotorIDs.kShooterFlywheelLeftMotorCANID,
            Constants.CANIDs.MotorIDs.kShooterFlywheelLeftFollowerCANID,
            "rio",
            Constants.EnabledSubsystems.kShooterLeft,
            true,
            InvertedValue.Clockwise_Positive,
            Seconds.of(0.05),
            Seconds.of(0.05),
            0.17345,
            0.0,
            0.0,
            0.11725,
            0.080935,
            0.0092333,
            Amps.of(70),
            Amps.of(50),
            true,
            true);

    public static final ShooterConfig rightConfig = new ShooterConfig(
            "RightShooter",
            Constants.CANIDs.MotorIDs.kShooterFlywheelRightMotorCANID,
            Constants.CANIDs.MotorIDs.kShooterFlywheelRightFollowerCANID,
            "rio",
            Constants.EnabledSubsystems.kShooterRight,
            true,
            InvertedValue.Clockwise_Positive,
            Seconds.of(0.05),
            Seconds.of(0.05),
            0.17345,
            0.0,
            0.0,
            0.11725,
            0.080935,
            0.0092333,
            Amps.of(70),
            Amps.of(50.0),
            true,
            true);
}
