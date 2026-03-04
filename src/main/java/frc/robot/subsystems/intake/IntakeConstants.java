package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularMomentum;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class IntakeConstants {

    public static final Distance kIntakeWidth = Inches.of(24);
    public static final Distance kIntakeExtension = Inches.of(10);
    public static final IntakeSide kIntakeSide = IntakeSide.FRONT;
    public static final int kIntakeCapacity = 50;

    public static class RollerConstants {
        // TODO: Fix incorrect Constants
        public static final double kIntakePercent = 0.4;
        public static final double kOuttakePercent = -0.6;
        public static final AngularVelocity kIntakeSpeed = RotationsPerSecond.of(50);
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
            public static final Current kStatorCurrentLimit = Amps.of(60);
            public static final double kPeakForwardTorque = 30;
            public static final double kPeakReverseTorque = -30;
        }
    }

    public static class ExtenderConstants {
        public static final int kExtenderTicksPerRevolution = 2048;
        public static final double kGearing = 54;
        public static final AngularMomentum kMOI = KilogramMetersSquaredPerSecond.of(0.5);
        public static final Distance kExtenderArmLength = Inches.of(12.0);
        public static final double kDownSpeed = 0.05;

        public static final Angle kExtenderStowAngle = Degrees.of(90.0);
        public static final Angle kExtenderIntakeAngle = Degrees.of(0);
        public static final Angle kExtenderMaxAngle = Degrees.of(90.0);
        public static final Angle kExtenderMinAngle = Degrees.of(0.0);
        public static final Angle kExtenderTolerance = Degrees.of(2);
        public static final Angle kExtenderSiftAngleOne = Degrees.of(50);
        public static final Angle kExtenderSiftAngleTwo = Degrees.of(5);

        public static class PIDF {
            public static final double kP = 5.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
        }

        public static class MotorConfig {
            public static final double kRampPeriod = 0.02;
            public static final Current kStatorCurrentLimitExtender = Amps.of(15);
        }
    }

    public static final boolean disabled = false;
}
