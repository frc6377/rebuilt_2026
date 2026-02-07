package frc.robot.subsystems.shooter.left;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Left shooter subsystem. */
public class LeftShooter extends SubsystemBase {
    private final LeftShooterIO io;
    private final LeftShooterIOInputsAutoLogged inputs = new LeftShooterIOInputsAutoLogged();

    // Tunable spin ratio
    private final LoggedNetworkNumber spinRatio =
            new LoggedNetworkNumber("LeftShooter/SpinRatio", LeftShooterConstants.defaultSpinRatio);

    // Setpoints
    private AngularVelocity flywheelSetpoint = RPM.of(0.0);
    private AngularVelocity spinSetpoint = RPM.of(0.0);

    // Failure state
    private boolean flywheelFailed = false;

    public LeftShooter(LeftShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LeftShooter", inputs);

        Logger.recordOutput("LeftShooter/Enabled", LeftShooterConstants.enabled);
        Logger.recordOutput("LeftShooter/FlywheelSetpoint", flywheelSetpoint);
        Logger.recordOutput("LeftShooter/SpinSetpoint", spinSetpoint);
        Logger.recordOutput("LeftShooter/SpinRatio", spinRatio.get());
        Logger.recordOutput("LeftShooter/FlywheelFailed", flywheelFailed);
    }

    /**
     * Set flywheel velocity. Also sets spin motor based on spin ratio.
     *
     * @param velocity Target velocity
     */
    public void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelSetpoint = velocity;
        double currentSpinRatio = spinRatio.get();
        spinSetpoint = RPM.of(velocity.in(RPM) * currentSpinRatio);

        if (LeftShooterConstants.enabled && !flywheelFailed) {
            io.setFlywheelVelocity(velocity);
            io.setSpinVelocity(spinSetpoint);
        } else {
            io.setFlywheelVelocity(RPM.of(0.0));
            io.setSpinVelocity(RPM.of(0.0));
        }
    }

    /** Set spin motor velocity directly. */
    public void setSpinVelocity(AngularVelocity velocity) {
        spinSetpoint = velocity;
        if (LeftShooterConstants.enabled && !flywheelFailed) {
            io.setSpinVelocity(velocity);
        } else {
            io.setSpinVelocity(RPM.of(0.0));
        }
    }

    /** Stop all motors. */
    public void stop() {
        flywheelSetpoint = RPM.of(0.0);
        spinSetpoint = RPM.of(0.0);
        io.stop();
    }

    /** Get current flywheel velocity. */
    public AngularVelocity getFlywheelVelocity() {
        return inputs.flywheelVelocity;
    }

    /** Check if flywheel is at target velocity. */
    @AutoLogOutput(key = "LeftShooter/AtTargetVelocity")
    public boolean atTargetVelocity() {
        AngularVelocity tolerance = ShooterConstants.flywheelVelocityTolerance;
        return flywheelFailed
                || Math.abs(inputs.flywheelVelocity.in(RPM) - flywheelSetpoint.in(RPM)) < tolerance.in(RPM);
    }

    public boolean isFailed() {
        return flywheelFailed;
    }

    public void resetFailureFlags() {
        flywheelFailed = false;
    }

    // ========== Command Factory Methods ==========

    public Command spinUpFlywheels(AngularVelocity velocity) {
        return Commands.run(() -> setFlywheelVelocity(velocity), this).withName("LeftSpinUp");
    }

    public Command spinUpFlywheels(Supplier<AngularVelocity> velocitySupplier) {
        return Commands.run(() -> setFlywheelVelocity(velocitySupplier.get()), this)
                .withName("LeftSpinUp");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("LeftStop");
    }

    public Command waitUntilReady() {
        return Commands.waitUntil(this::atTargetVelocity).withName("LeftWaitUntilReady");
    }

    public Command spinUpAndWait(AngularVelocity velocity) {
        return spinUpFlywheels(velocity).andThen(waitUntilReady()).withName("LeftSpinUpAndWait");
    }
}
