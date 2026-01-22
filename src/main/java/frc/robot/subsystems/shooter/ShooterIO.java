package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean shooterMotorConnected = true;
        public double shooterPositionRad = 0.0;
        public double shooterVelocityRadPerSec = 0.0;
        public double shooterAppliedVolts = 0.0;
        public double shooterCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Run the shooter motor at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Run the shooter motor at the specified velocity. */
    public default void setVelocity(AngularVelocity velocity) {}

    /** Stop the shooter motor. */
    public default void stop() {}
}
