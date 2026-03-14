package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.TimedRobot;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class IntakeConstants {

    public static final Distance kIntakeWidth = Inches.of(24);
    public static final Distance kIntakeExtension = Inches.of(10);
    public static final IntakeSide kIntakeSide = IntakeSide.FRONT;
    public static final int kIntakeCapacity = 50;

    public static class RollerConstants {
        // TODO: Fix incorrect Constants
        public static final double kIntakePercent = 1;
        public static final double kOuttakePercent = -0.6;
        public static final AngularVelocity kIntakeSpeed = RadiansPerSecond.of(400);
        public static final AngularVelocity kOuttakeSpeed = RotationsPerSecond.of(50);

        public static class PIDF {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
        }

        public static class MotorConfig {
            public static final double kRampPeriod = 0.02;
            public static final Current kStatorCurrentLimit = Amps.of(70);
            public static final InvertedValue kInvertedReal = InvertedValue.CounterClockwise_Positive;
            public static final InvertedValue kInvertedSim = InvertedValue.Clockwise_Positive;
            public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        }
    }

    public static class ExtenderConstants {

        public static final double kGearing = 1;
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(1.1);
        public static final Distance kExtenderArmLength = Inches.of(13.75);
        public static final double kDownSpeed = 0.05;

        public static final Angle kExtenderStowAngle = Degrees.of(0.0);
        public static final Angle kExtenderIntakeAngle = Degrees.of(130.0);
        public static final Angle kExtenderTolerance = Degrees.of(2);
        public static final Angle kExtenderSiftAngleOne = Degrees.of(120);
        public static final Angle kExtenderSiftAngleTwo = Degrees.of(90);
        public static final Angle kExtenderCustomAngleOne = Degrees.of(70.0);
        public static final Angle kExtenderCustomAngleTwo = Degrees.of(20.0);
        public static final Angle kExtenderZeroAngle = Degrees.of(15);

        public static class PIDF {
            public static final double kP = 0.005;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
        }

        public static class MotorConfig {
            public static final double kRampPeriod = TimedRobot.kDefaultPeriod;
            public static final Current kStatorCurrentLimitExtender = Amps.of(15);
            public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
            public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
        }
    }
}
