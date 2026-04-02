package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface BaseShooterIO {
    @AutoLog
    class BaseShooterIOInputs {
        public AngularVelocity flywheelVelocity = RPM.of(0.0);
        public Voltage flywheelAppliedVoltage = Volts.of(0.0);
        public Current flywheelCurrent = Amps.of(0.0);
        public Temperature flywheelTemp = Celsius.of(0.0);
        public AngularVelocity flywheelVelocity2 = RPM.of(0.0);
        public Voltage flywheelAppliedVoltage2 = Volts.of(0.0);
        public Current flywheelCurrent2 = Amps.of(0.0);
        public Temperature flywheelTemp2 = Celsius.of(0.0);
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(BaseShooterIOInputs inputs) {}

    /** Set flywheel velocity. */
    default void setFlywheelVelocity(AngularVelocity velocity) {}

    /** Set flywheel voltage. */
    default void setFlywheelVoltage(Voltage voltage) {}

    /** Stop all motors. */
    default void stop() {}
}
