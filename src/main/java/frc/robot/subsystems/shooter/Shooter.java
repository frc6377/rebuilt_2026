package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    /** Creates a new Shooter. */
    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    /** Run the shooter at the specified voltage. */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    /** Run the shooter at the specified velocity. */
    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    /** Stop the shooter. */
    public void stop() {
        io.stop();
    }

    /** Returns the current velocity in rad/s. */
    public double getVelocityRadPerSec() {
        return inputs.shooterVelocityRadPerSec;
    }
}
