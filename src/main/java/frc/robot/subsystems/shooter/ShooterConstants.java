package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.*;
import java.util.HashMap;

public record ShooterConstants(
        boolean kSotfEnabled,
        CalculationMode kDefaultCalculationMode,
        boolean kManualShootingEnabled,
        int kManualShootingSpeedRPM,
        Angle kFixedHoodAngle,
        AngularVelocity kMaxFlywheelVelocity,
        AngularVelocity kFlywheelVelocityTolerance,
        Angle kHeadingTolerance,
        AngularVelocity kMaxVelocityDifference,
        double kAccelMismatchToleranceRPM,
        int kAccelMismatchCycles,
        Distance shooterHeight,
        Distance shooterOffsetXLeft,
        Distance shooterOffsetYLeft,
        Distance shooterOffsetXRight,
        Distance shooterOffsetYRight,
        Distance flywheelRadius,
        double launchEfficiency,
        AngularVelocity minShootingFlywheelVelocity,
        AngularVelocity maxShootingFlywheelVelocity,
        LinearAcceleration gravity,
        AngularVelocity kDefaultUnjamVelocity,
        double defaultMaxHeightFeet,
        double defaultTargetHeightFeet,
        double defaultRpmMultiplier,
        double defaultBenchModeEnabled,
        double defaultBenchModeDistanceMeters,
        double offsetM,
        InterpolatingDoubleTreeMap distanceToAngularVelocityDouMapRPM,
        HashMap<Distance, AngularVelocity> distanceToAngularVelocityMapRPM,
        ShooterConfig leftConfig,
        ShooterConfig rightConfig) {
    public enum CalculationMode {
        PHYSICS,
        DOU_INTERPOLATION
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
            boolean flywheelCurrentLimitSupplyEnable,
            MotorOutputConfigs outputConfigs) {}
}
