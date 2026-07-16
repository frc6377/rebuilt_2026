package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterConfig;
import frc.robot.util.TunablePIDController.PIDConfig;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class BaseIntakeConstants {
    public static final IntakeConstants DATA;

    static {
        ShooterConfig rollerConfig = new ShooterConfig(
                "Roller",
                Constants.CANIDs.MotorIDs.kRollerLeaderMotorID,
                Constants.CANIDs.MotorIDs.kRollerFollowerMotorID,
                "rio",
                Constants.EnabledSubsystems.kRoller,
                true, // kfollowerEnabled
                InvertedValue.CounterClockwise_Positive,
                Seconds.of(0.02),
                Seconds.of(0.02),
                0.19188,
                0.0,
                0.0,
                0.11058,
                0.122298,
                0.087816,
                Amps.of(60),
                Amps.of(50),
                true,
                true,
                new MotorOutputConfigs().withPeakForwardDutyCycle(1).withPeakReverseDutyCycle(-1));

        DATA = new IntakeConstants(
                Inches.of(24), // kIntakeWidth
                Inches.of(10), // kIntakeExtension
                IntakeSide.FRONT, // kIntakeSide
                50, // kIntakeCapacity
                true, // rollerFollowerEnabled
                true, // rollerIdleEnabled
                1.0, // rollerIntakePercent
                -0.6, // rollerOuttakePercent
                0.1, // rollerIdlePercent
                RPM.of(2500), // rollerIntakeSpeed
                RPM.of(-1800), // rollerOuttakeSpeed
                RPM.of(100), // rollerIdleSpeed
                0.19188, // rollerKP
                0.0, // rollerKI
                0.0, // rollerKD
                0.122298, // rollerKS
                0.11058, // rollerKV
                0.087816, // rollerKA
                0.02, // rollerRampPeriod
                Amps.of(60), // rollerStatorCurrentLimit
                Amps.of(50), // rollerSupplyCurrentLimit
                InvertedValue.CounterClockwise_Positive, // rollerInverted
                NeutralModeValue.Coast, // rollerNeutralMode
                MotorAlignmentValue.Opposed, // rollerFollowerInverted
                rollerConfig,
                true, // extenderFloatEnabled
                1.0, // extenderGearing
                KilogramSquareMeters.of(1.5), // extenderMOI
                Inches.of(12.0), // extenderArmLength
                0.05, // extenderDownSpeed
                Degrees.of(0).plus(Degrees.of(175)), // extenderStowAngle
                Degrees.of(97).plus(Degrees.of(180)), // extenderIntakeAngle
                Degrees.of(5), // extenderTolerance
                Degrees.of(0.0).plus(Degrees.of(180)), // extenderSiftAngleOne
                Degrees.of(97.0).plus(Degrees.of(180)), // extenderSiftAngleTwo
                Degrees.of(45.0).plus(Degrees.of(180)), // extenderCustomAngleOne
                Degrees.of(60.0).plus(Degrees.of(180)), // extenderCustomAngleTwo
                Degrees.of(50).plus(Degrees.of(180)), // extenderFloatLimit
                Degrees.of(-56.70141).plus(Degrees.of(180)), // extenderZeroAngle
                new TrapezoidProfile.Constraints(100000, 7500), // extenderConstraints
                new TrapezoidProfile.Constraints(100000, 7500), // extenderSiftConstraints
                Seconds.of(0.5), // extenderSiftTimeout
                new PIDConfig(0.005, 0.0, 0.00001), // extenderNormalPID
                new PIDConfig(0.00, 0.0, 0.00001), // extenderFloatPID
                new PIDConfig(0.007, 0.0, 0.00001), // extenderBabyPID
                4.0, // extenderRampPeriod
                Amps.of(30), // extenderStatorCurrentLimit
                Amps.of(25), // extenderSupplyCurrentLimit
                InvertedValue.Clockwise_Positive, // extenderInverted
                NeutralModeValue.Brake // extenderNeutralMode
                );
    }
}
