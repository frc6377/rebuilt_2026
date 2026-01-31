package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularMomentum;

public class IntakeConstants {
    public class MotorIDs {
        // TODO: Fix incorrect Constants
        // public static final int PIVOT_MOTOR_ID = 0;
        public static final int ROLLER_MOTOR_ID = 7;
    }

    public class SensorIDs {
        // TODO: Fix incorrect Constants
        public static final int PIVOT_ENCODER_ID = 1;
    }

    public class RollerConstants {
        // TODO: Fix incorrect Constants
        public static final double INTAKE_SPEED = -1;
        public static final double OUTAKE_SPEED = 1;

        public static final AngularMomentum ROLLER_MOI = KilogramMetersSquaredPerSecond.of(0.00032);
        public static final double ROLLER_GEARING = 1.0;
    }

    public class PivotConstants {
        // TODO: Fix incorrect Constants
        public class PivotPID {
            public static final double P = 0.0;
            public static final double I = 0.0;
            public static final double D = 0.0;
        }

        public static class PivotFW {
            public static final double S = 0.0;
            public static final double G = 0.0;
            public static final double V = 0.0;
        }

        public static final Angle intakeAngle = Degrees.of(0);
    }
}
