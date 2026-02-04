package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularMomentum;
import edu.wpi.first.units.measure.Distance;

public class IntakeConstants {
    public class RollerConstants {
        // TODO: Fix incorrect Constants
        public static final double kIntakeSpeed = 0.5;
        public static final double kOuttakeSpeed = -0.5;
    }

    public class ExtenderConstants {
        public static final int kExtenderTicksPerRevolution = 2048;
        public static final double kGearing = 10.0;
        public static final AngularMomentum kMOI = KilogramMetersSquaredPerSecond.of(0.5);
        public static final Distance kExtenderArmLength = Inches.of(12.0);

        public static final Angle kExtenderStowAngle = Degrees.of(25.0);
        public static final Angle kExtenderIntakeAngle = Degrees.of(5.0);
        public static final Angle kExtenderMaxAngle = Degrees.of(45);
        public static final Angle kExtenderMinAngle = Degrees.of(0.0);
        public static final Angle kExtenderTolerance = Degrees.of(2.0);
        public static final Angle kExtenderSiftAngleOne = Degrees.of(13);
        public static final Angle kExtenderSiftAngleTwo = Degrees.of(25);

        public class PIDF {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kF = 0.0;
        }
    }
}
