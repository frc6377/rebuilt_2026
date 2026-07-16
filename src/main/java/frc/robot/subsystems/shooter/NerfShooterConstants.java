package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

public class NerfShooterConstants {
    public static final ShooterConstants DATA;

    static {
        ShooterConstants base = BaseShooterConstants.DATA;

        // Nerf theconfigs
        ShooterConstants.ShooterConfig nerfedLeftConfig = new ShooterConstants.ShooterConfig(
                base.leftConfig().name(),
                base.leftConfig().flywheelLeaderId(),
                base.leftConfig().flywheelFollowerId(),
                base.leftConfig().canBusName(),
                base.leftConfig().enabled(),
                base.leftConfig().followerEnabled(),
                base.leftConfig().flywheelInverted(),
                base.leftConfig().flywheelOpenLoopRamp(),
                base.leftConfig().flywheelClosedLoopRamp(),
                base.leftConfig().flywheelKP(),
                base.leftConfig().flywheelKI(),
                base.leftConfig().flywheelKD(),
                base.leftConfig().flywheelKV(),
                base.leftConfig().flywheelKS(),
                base.leftConfig().flywheelKA(),
                base.leftConfig().flywheelCurrentLimitStator(),
                base.leftConfig().flywheelCurrentLimitSupply(),
                base.leftConfig().flywheelCurrentLimitStatorEnable(),
                base.leftConfig().flywheelCurrentLimitSupplyEnable(),
                base.leftConfig().outputConfigs());

        ShooterConstants.ShooterConfig nerfedRightConfig = new ShooterConstants.ShooterConfig(
                base.rightConfig().name(),
                base.rightConfig().flywheelLeaderId(),
                base.rightConfig().flywheelFollowerId(),
                base.rightConfig().canBusName(),
                base.rightConfig().enabled(),
                base.rightConfig().followerEnabled(),
                base.rightConfig().flywheelInverted(),
                base.rightConfig().flywheelOpenLoopRamp(),
                base.rightConfig().flywheelClosedLoopRamp(),
                base.rightConfig().flywheelKP(),
                base.rightConfig().flywheelKI(),
                base.rightConfig().flywheelKD(),
                base.rightConfig().flywheelKV(),
                base.rightConfig().flywheelKS(),
                base.rightConfig().flywheelKA(),
                base.rightConfig().flywheelCurrentLimitStator(),
                base.rightConfig().flywheelCurrentLimitSupply(),
                base.rightConfig().flywheelCurrentLimitStatorEnable(),
                base.rightConfig().flywheelCurrentLimitSupplyEnable(),
                base.rightConfig().outputConfigs());

        DATA = new ShooterConstants(
                base.kSotfEnabled(),
                base.kDefaultCalculationMode(),
                base.kManualShootingEnabled(),
                1800, // kManualShootingSpeedRPM (nerfed from 3600)
                base.kFixedHoodAngle(),
                RotationsPerSecond.of(50.0), // kMaxFlywheelVelocity (nerfed from 100.0)
                base.kFlywheelVelocityTolerance(),
                base.kHeadingTolerance(),
                base.kMaxVelocityDifference(),
                base.kAccelMismatchToleranceRPM(),
                base.kAccelMismatchCycles(),
                base.shooterHeight(),
                base.shooterOffsetXLeft(),
                base.shooterOffsetYLeft(),
                base.shooterOffsetXRight(),
                base.shooterOffsetYRight(),
                base.flywheelRadius(),
                base.launchEfficiency(),
                base.minShootingFlywheelVelocity(),
                RPM.of(3000.0), // maxShootingFlywheelVelocity (nerfed from 6000.0)
                base.gravity(),
                base.kDefaultUnjamVelocity(),
                base.defaultMaxHeightFeet(),
                base.defaultTargetHeightFeet(),
                base.defaultRpmMultiplier(),
                base.defaultBenchModeEnabled(),
                base.defaultBenchModeDistanceMeters(),
                base.offsetM(),
                base.distanceToAngularVelocityDouMapRPM(),
                nerfedLeftConfig,
                nerfedRightConfig);
    }
}
