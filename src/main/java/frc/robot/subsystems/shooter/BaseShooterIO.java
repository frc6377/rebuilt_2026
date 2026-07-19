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
        public AngularVelocity followerFlywheelVelocity = RPM.of(0.0);
        public Voltage followerFlywheelAppliedVoltage = Volts.of(0.0);
        public Current followerFlywheelCurrent = Amps.of(0.0);
        public Temperature followerFlywheelTemp = Celsius.of(0.0);
        public boolean supplyCurrentValid = false;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(BaseShooterIOInputs inputs) {}

    /** Set flywheel velocity. */
    default void setFlywheelVelocity(AngularVelocity velocity) {}

    /** Set flywheel voltage. */
    default void setFlywheelVoltage(Voltage voltage) {}

    /** Stop all motors. */
    default void stop() {}

    /** Requests a dynamic supply-current limit when this IO implementation explicitly supports it. */
    default void setSupplyCurrentLimit(double currentLimitAmps) {}
}
