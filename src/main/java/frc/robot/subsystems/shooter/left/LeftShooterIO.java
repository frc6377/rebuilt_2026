package frc.robot.subsystems.shooter.left;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface LeftShooterIO {
    @AutoLog
    public static class LeftShooterIOInputs {
        public AngularVelocity flywheelVelocity = RPM.of(0.0);
        public Voltage flywheelAppliedVoltage = Volts.of(0.0);
        public Current flywheelCurrent = Amps.of(0.0);
        public Temperature flywheelTemp = Celsius.of(0.0);

        public AngularVelocity spinVelocity = RPM.of(0.0);
        public Voltage spinAppliedVoltage = Volts.of(0.0);
        public Current spinCurrent = Amps.of(0.0);
        public Temperature spinTemp = Celsius.of(0.0);
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(LeftShooterIOInputs inputs) {}

    /** Set flywheel velocity. */
    default void setFlywheelVelocity(AngularVelocity velocity) {}

    /** Set spin motor velocity. */
    default void setSpinVelocity(AngularVelocity velocity) {}

    /** Stop all motors. */
    default void stop() {}
}
