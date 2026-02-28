package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;

public class IndexerConstants {

    public static final double kCollectorSpeed = 0.5; // Adjust speed as needed
    public static final AngularVelocity kCollectorRPM = RPM.of(300); // Adjust RPM as needed

    public static final String canBus = "rio";

    public final class MotorConfigurationConfigs {
        public static final double VoltageClosedLoopRampPeriod = 0.02;
        public static final double PeakForwardTorqueCurrent = 40;
        public static final double PeakReverseTorqueCurrent = 40;
        public static final InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue MotorNeutralMode = NeutralModeValue.Brake;
    }

    public final class PID {
        public static final double kP = 0.0; // Proportional gain
        public static final double kI = 0.0; // Integral gain
        public static final double kD = 0.0; // Derivative gain
    }

    public final class SimPID {
        public static final double kP = 0.1; // Proportional gain
        public static final double kI = 0.0; // Integral gain
        public static final double kD = 0.0; // Derivative gain
    }

    public final class Feedforward {
        public static final double NOMINAL_VOLTAGE = RobotController.getBatteryVoltage(); // Volts
        public static final double kS = 0.0; // Static friction (V)
        public static final double kA = 0.0; // Acceleration (V/(RPM/s))
    }

    public final class SimConstants {
        public static final double ROLLER_MASS_KG = 0.25;
        public static final double ROLLER_RADIUS_M = 0.02;
        public static final double ROLLER_MOI = 0.5 * ROLLER_MASS_KG * ROLLER_RADIUS_M * ROLLER_RADIUS_M;
        public static final double ROLLER_GEARING = 1.0;
    }
}
