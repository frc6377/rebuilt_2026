package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterConfig;
import frc.robot.util.TunablePIDController.PIDConfig;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class IntakeConstants {

    public static final Distance kIntakeWidth = Inches.of(24);
    public static final Distance kIntakeExtension = Inches.of(10);
    public static final IntakeSide kIntakeSide = IntakeSide.FRONT;
    public static final int kIntakeCapacity = 50;

    public static class RollerConstants {

        // for normal roller io

        public static final boolean kfollowerEnabled = true;
        public static final boolean kIdleEnabled = false;

        // for normal roller io
        public static final double kIntakePercent = 1;
        public static final double kOuttakePercent = -0.6;
        public static final double kIdlePercent = 0.1;

        // for pid roller io
        public static final AngularVelocity kIntakeSpeed = RPM.of(800);
        public static final AngularVelocity kOuttakeSpeed = RPM.of(800);
        public static final AngularVelocity kIdleSpeed = RPM.of(100);

        public static class PIDF {
            public static final double kP = 0.03;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
        }

        public static class MotorConfig {
            public static final double kRampPeriod = 0.02;
            public static final Current kStatorCurrentLimit = Amps.of(70);
            public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
            public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;

            public static final MotorAlignmentValue kFollowerInverted = MotorAlignmentValue.Opposed;
        }

        public static final ShooterConfig rollerConfig = new ShooterConfig(
                "Roller",
                Constants.CANIDs.MotorIDs.kRollerLeaderMotorID,
                Constants.CANIDs.MotorIDs.kRollerFollowerMotorID,
                "rio",
                Constants.EnabledSubsystems.kRoller,
                kfollowerEnabled, // kfollowerEnabled is false below
                MotorConfig.kInverted,
                Seconds.of(MotorConfig.kRampPeriod),
                Seconds.of(MotorConfig.kRampPeriod),
                PIDF.kP,
                PIDF.kI,
                PIDF.kD,
                PIDF.kV,
                PIDF.kS,
                PIDF.kA,
                MotorConfig.kStatorCurrentLimit,
                Amps.of(90), // default supply current limit
                true,
                true);
    }

    public static class ExtenderConstants {

        public static final boolean floatEnabled = false;

        public static final double kGearing = 1;
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(1.5);
        public static final Distance kExtenderArmLength = Inches.of(12.0);
        public static final double kDownSpeed = 0.05;

        public static final Angle kExtenderStowAngle = Degrees.of(0);
        public static final Angle kExtenderIntakeAngle = Degrees.of(97.0);
        public static final Angle kExtenderTolerance = Degrees.of(2.5);
        public static final Angle kExtenderSiftAngleOne = Degrees.of(5.0);
        public static final Angle kExtenderSiftAngleTwo = Degrees.of(90.0);
        public static final Angle kExtenderCustomAngleOne = Degrees.of(45.0);
        public static final Angle kExtenderCustomAngleTwo = Degrees.of(60.0);
        public static final Angle kExtenderFloatLimit = Degrees.of(50);
        public static final Angle kExtenderZeroAngle = Degrees.of(-169);

        public static final Time kSiftTimeout = Seconds.of(0.5);

        public static class PIDF {

            public static final PIDConfig normalPID = new PIDConfig(0.05, 0.0, 0.00001);

            public static final PIDConfig floatPID = new PIDConfig(0.01, 0.0, 0.00001);
        }

        public static class MotorConfig {
            public static final double kRampPeriod = 0.5;
            public static final Current kStatorCurrentLimitExtender = Amps.of(30);
            public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
            public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        }
    }
}
