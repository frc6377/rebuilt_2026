package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SuperstructureConstants {
    // Trajectory target heights (tunable via NetworkTables)
    public static final LoggedNetworkNumber maxHeightFeet =
            new LoggedNetworkNumber("Shooting/MaxHeightFeet", ShooterConstants.defaultMaxHeightFeet);
    public static final LoggedNetworkNumber targetHeightFeet =
            new LoggedNetworkNumber("Shooting/TargetHeightFeet", ShooterConstants.defaultTargetHeightFeet);

    // Fine-tuning offsets
    public static final LoggedNetworkNumber hoodAngleOffset =
            new LoggedNetworkNumber("Shooting/HoodAngleOffset", ShooterConstants.defaultHoodAngleOffset);
    public static final LoggedNetworkNumber rpmMultiplier =
            new LoggedNetworkNumber("Shooting/RPMMultiplier", ShooterConstants.defaultRpmMultiplier);
    public static final LoggedNetworkNumber calculationMode =
            new LoggedNetworkNumber("Shooting/CalculationMode", ShooterConstants.kDefaultCalculationMode.ordinal());
    public static final LoggedNetworkNumber manualShootingSpeedRPM =
            new LoggedNetworkNumber("Shooting/ManualShootingSpeedRPM", ShooterConstants.kManualShootingSpeedRPM);
    public static final LoggedNetworkNumber manualShootingEnabled = new LoggedNetworkNumber(
            "Shooting/ManualShootingEnabled", ShooterConstants.kManualShootingEnabled ? 1.0 : 0.0);

    // Testing / Bench Mode
    public static final LoggedNetworkNumber benchModeEnabled =
            new LoggedNetworkNumber("Shooting/BenchMode/Enabled", ShooterConstants.defaultBenchModeEnabled);
    public static final LoggedNetworkNumber benchModeDistanceFeet = new LoggedNetworkNumber(
            "Shooting/BenchMode/DistanceMeters", ShooterConstants.defaultBenchModeDistanceMeters);
}
