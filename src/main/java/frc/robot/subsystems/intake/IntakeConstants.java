package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public static final Distance kIntakeWidth = Inches.of(30);
    public static final Distance kIntakeExtension = Inches.of(2);
    public static final IntakeSide kIntakeSide = IntakeSide.FRONT;
    public static final int kIntakeCapacity = 60;

    public static class RollerConstants {

        // for normal roller io

        public static final boolean kfollowerEnabled = true;
        public static final boolean kIdleEnabled = true;

        // for normal roller io
        public static final double kIntakePercent = 1;
        public static final double kOuttakePercent = -0.6;
        public static final double kIdlePercent = 0.1;

        // for pid roller io
        public static final AngularVelocity kIntakeSpeed = RPM.of(2500); // NEVER GO HIGHER THAN THIS - JOSH
        public static final AngularVelocity kOuttakeSpeed = RPM.of(-1800);
        public static final AngularVelocity kIdleSpeed = RPM.of(100);

        public static class PIDF {
            public static final double kP = 0.19188;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.122298;
            public static final double kV = 0.11058;
            public static final double kA = 0.087816;
        }

        public static class MotorConfig {
            public static final double kRampPeriod = 0.02;
            public static final Current kStatorCurrentLimit = Amps.of(60);
            public static final Current kSupplyCurrentLimit = Amps.of(50);
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
                MotorConfig.kSupplyCurrentLimit,
                true,
                true,
                new MotorOutputConfigs().withPeakForwardDutyCycle(1).withPeakReverseDutyCycle(-1));
    }

    public static class ExtenderConstants {

        public static final boolean floatEnabled = true;

        public static final double kGearing = 1;
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(1.5);
        public static final Distance kExtenderArmLength = Inches.of(8.1);
        public static final double kDownSpeed = 0.05;

        public static final Angle kExtenderStowAngle = Degrees.of(0).plus(Degrees.of(175));
        public static final Angle kExtenderIntakeAngle = Degrees.of(97).plus(Degrees.of(180));
        public static final Angle kExtenderTolerance = Degrees.of(5);
        public static final Angle kExtenderSiftAngleOne = Degrees.of(0.0).plus(Degrees.of(180));
        public static final Angle kExtenderSiftAngleTwo = Degrees.of(97.0).plus(Degrees.of(180));
        public static final Angle kExtenderCustomAngleOne = Degrees.of(45.0).plus(Degrees.of(180));
        public static final Angle kExtenderCustomAngleTwo = Degrees.of(60.0).plus(Degrees.of(180));
        public static final Angle kExtenderFloatLimit = Degrees.of(50).plus(Degrees.of(180));
        public static final Angle kExtenderZeroAngle = Degrees.of(-56.70141).plus(Degrees.of(180));
        public static final TrapezoidProfile.Constraints kExtenderConstraints =
                new TrapezoidProfile.Constraints(100000, 7500);
        public static final TrapezoidProfile.Constraints SIFT_CONSTRAINTS =
                new TrapezoidProfile.Constraints(100000, 7500);

        public static final Time kSiftTimeout = Seconds.of(0.5);

        public static class ComponentPoses {
            public static final Pose3d topBarsPose =
                    new Pose3d(new Translation3d(0.111, -0.315, 0.38), new Rotation3d(0, 0, 0));
            public static final Pose3d bottomBarsPose =
                    new Pose3d(new Translation3d(0.288, -0.31, 0.22), new Rotation3d(0, 0, 0));
            public static final Pose3d intakePose =
                    new Pose3d(new Translation3d(0.289, -0.305, 0.46), new Rotation3d(0, 0, 0));
        }

        public static class FourBarLinkage {
            public static final Distance kBottomBarLength = kExtenderArmLength;
            public static final Distance kTopBarLength = Inches.of(7.9);
            public static final Angle kBottomBarAngleAboveFloorAtExtended = Degrees.of(28.65);
            public static final Angle kTopBarAngleAboveFloorAtExtended = Degrees.of(25.78);
        }

        public static class PIDF {

            public static final PIDConfig normalPID = new PIDConfig(0.005, 0.0, 0.00001);

            public static final PIDConfig floatPID = new PIDConfig(0.00, 0.0, 0.00001);

            public static final PIDConfig babyPID = new PIDConfig(0.007, 0.0, 0.00001);
        }

        public static class MotorConfig {
            public static final double kRampPeriod = 4;
            public static final Current kStatorCurrentLimitExtender = Amps.of(30);
            public static final Current kSupplyCurrentLimitExtender = Amps.of(25);
            public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
            public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        }
    }
}
