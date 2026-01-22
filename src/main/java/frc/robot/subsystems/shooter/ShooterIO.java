package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean shooterMotorConnected = true;
        public Angle shooterPosition = Radians.of(0.0);
        public AngularVelocity shooterVelocity = RadiansPerSecond.of(0.0);
        public Voltage shooterAppliedVolts = Volts.of(0);
        public Current shooterCurrent = Amps.of(0);
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Run the shooter motor at the specified voltage. */
    public default void setVoltage(Voltage volts) {}

    /** Run the shooter motor at the specified velocity. */
    public default void setVelocity(AngularVelocity velocity) {}

    /** Stop the shooter motor. */
    public default void stop() {}
}
