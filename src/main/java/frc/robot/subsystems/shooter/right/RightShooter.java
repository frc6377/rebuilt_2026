package frc.robot.subsystems.shooter.right;

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

/** Right shooter subsystem. */
public class RightShooter extends SubsystemBase {
    private final RightShooterIO io;
    private final RightShooterIOInputsAutoLogged inputs = new RightShooterIOInputsAutoLogged();

    // Tunable spin ratio
    private final LoggedNetworkNumber spinRatio =
            new LoggedNetworkNumber("RightShooter/SpinRatio", RightShooterConstants.defaultSpinRatio);

    // Setpoints
    private AngularVelocity flywheelSetpoint = RPM.of(0.0);
    private AngularVelocity spinSetpoint = RPM.of(0.0);

    // Failure state
    private boolean flywheelFailed = false;

    public RightShooter(RightShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("RightShooter", inputs);

        Logger.recordOutput("RightShooter/Enabled", RightShooterConstants.enabled);
        Logger.recordOutput("RightShooter/FlywheelSetpoint", flywheelSetpoint);
        Logger.recordOutput("RightShooter/SpinSetpoint", spinSetpoint);
        Logger.recordOutput("RightShooter/SpinRatio", spinRatio.get());
        Logger.recordOutput("RightShooter/FlywheelFailed", flywheelFailed);
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

        if (RightShooterConstants.enabled && !flywheelFailed) {
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
        if (RightShooterConstants.enabled && !flywheelFailed) {
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
    @AutoLogOutput(key = "RightShooter/AtTargetVelocity")
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
        return Commands.run(() -> setFlywheelVelocity(velocity), this).withName("RightSpinUp");
    }

    public Command spinUpFlywheels(Supplier<AngularVelocity> velocitySupplier) {
        return Commands.run(() -> setFlywheelVelocity(velocitySupplier.get()), this)
                .withName("RightSpinUp");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("RightStop");
    }

    public Command waitUntilReady() {
        return Commands.waitUntil(this::atTargetVelocity).withName("RightWaitUntilReady");
    }

    public Command spinUpAndWait(AngularVelocity velocity) {
        return spinUpFlywheels(velocity).andThen(waitUntilReady()).withName("RightSpinUpAndWait");
    }
}
