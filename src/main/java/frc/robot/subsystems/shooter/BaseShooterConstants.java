package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import java.util.HashMap;

public class BaseShooterConstants {
    public static final ShooterConstants DATA;

    static {
        HashMap<Distance, AngularVelocity> distanceToAngularVelocity = new HashMap<>();
        distanceToAngularVelocity.put(Meters.of(0.0), RPM.of(1700));
        distanceToAngularVelocity.put(Meters.of(3.18), RPM.of(3600));
        distanceToAngularVelocity.put(Meters.of(4.6), RPM.of(4230));
        distanceToAngularVelocity.put(Meters.of(3.75), RPM.of(4000));
        distanceToAngularVelocity.put(Meters.of(1), RPM.of(2550));
        distanceToAngularVelocity.put(Meters.of(6), RPM.of(4500));

        double offsetM = 0.5969;
        InterpolatingDoubleTreeMap distanceToAngularVelocityDouMapRPM = new InterpolatingDoubleTreeMap();
        for (Distance distance : distanceToAngularVelocity.keySet()) {
            distanceToAngularVelocityDouMapRPM.put(
                    distance.in(Meters) + offsetM,
                    distanceToAngularVelocity.get(distance).in(RPM));
        }

        ShooterConstants.ShooterConfig leftConfig = new ShooterConstants.ShooterConfig(
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
                true,
                new MotorOutputConfigs().withPeakForwardDutyCycle(1).withPeakReverseDutyCycle(-0.05));

        ShooterConstants.ShooterConfig rightConfig = new ShooterConstants.ShooterConfig(
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
                true,
                new MotorOutputConfigs().withPeakForwardDutyCycle(1).withPeakReverseDutyCycle(-0.05));

        DATA = new ShooterConstants(
                false, // kSotfEnabled
                ShooterConstants.CalculationMode.DOU_INTERPOLATION, // kDefaultCalculationMode
                false, // kManualShootingEnabled
                3600, // kManualShootingSpeedRPM
                Degrees.of(60), // kFixedHoodAngle
                RotationsPerSecond.of(100.0), // kMaxFlywheelVelocity
                RPM.of(150.0), // kFlywheelVelocityTolerance
                Degrees.of(5), // kHeadingTolerance
                RotationsPerSecond.of(500.0 / 60.0), // kMaxVelocityDifference
                1000000, // kAccelMismatchToleranceRPM
                100000000, // kAccelMismatchCycles
                Meters.of(0.5), // shooterHeight
                Meters.of(0.3), // shooterOffsetXLeft
                Meters.of(0.0), // shooterOffsetYLeft
                Meters.of(0.3), // shooterOffsetXRight
                Meters.of(0.0), // shooterOffsetYRight
                Inches.of(2), // flywheelRadius
                0.75, // launchEfficiency
                RPM.of(1500.0), // minShootingFlywheelVelocity
                RPM.of(6000.0), // maxShootingFlywheelVelocity
                MetersPerSecondPerSecond.of(11), // gravity
                RPM.of(-1500), // kDefaultUnjamVelocity
                8.0, // defaultMaxHeightFeet
                6.0, // defaultTargetHeightFeet
                1.0, // defaultRpmMultiplier
                0.0, // defaultBenchModeEnabled
                10.0, // defaultBenchModeDistanceMeters
                offsetM,
                distanceToAngularVelocityDouMapRPM,
                distanceToAngularVelocity,
                leftConfig,
                rightConfig);
    }
}
