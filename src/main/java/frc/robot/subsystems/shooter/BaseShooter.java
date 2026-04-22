package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;

import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Base shooter subsystem. */
public class BaseShooter extends SubsystemBase {
    private final BaseShooterIO io;
    private final BaseShooterIOInputsAutoLogged inputs = new BaseShooterIOInputsAutoLogged();
    private final ShooterConstants.ShooterConfig config;

    private final @NotNull SysIdRoutine sysIdRoutine;

    // Setpoints
    private AngularVelocity flywheelSetpoint = RPM.of(0.0);

    // Failure state
    private boolean flywheelFailed = false;

    public BaseShooter(@NotNull BaseShooterIO io, ShooterConstants.ShooterConfig config) {
        this.io = io;
        this.config = config;

        this.sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(1), // Reduce dynamic step voltage to 4 to prevent brownout
                        Seconds.of(4), // Use default timeout (10 s)
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(io::setFlywheelVoltage, null, this));
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs(this.config.name(), this.inputs);
        Logger.recordOutput(this.config.name() + "/Enabled", this.config.enabled());
        Logger.recordOutput(this.config.name() + "/FollowerEnabled", this.config.followerEnabled());
        Logger.recordOutput(this.config.name() + "/FlywheelFailed", this.flywheelFailed);
        Logger.recordOutput(this.config.name() + "/FlywheelSetpoint", this.flywheelSetpoint);
        if (null != getCurrentCommand()) {
            Logger.recordOutput(
                    this.config.name() + "/CurrentCommand", this.getCurrentCommand().getName());
        } else {
            Logger.recordOutput(this.config.name() + "/CurrentCommand", "None");
        }
    }

    /**
     * Set flywheel velocity.
     *
     * @param velocity Target velocity
     */
    public void setFlywheelVelocity(AngularVelocity velocity) {
        this.flywheelSetpoint = velocity;

        if (this.config.enabled() && !this.flywheelFailed) {
            this.io.setFlywheelVelocity(velocity);
        } else {
            this.io.setFlywheelVelocity(RPM.of(0.0));
        }
    }

    /** Stop all motors. */
    public void stop() {
        this.flywheelSetpoint = RPM.of(0.0);
        this.io.stop();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.dynamic(direction);
    }

    /** Get current flywheel velocity. */
    public AngularVelocity getFlywheelVelocity() {
        return this.inputs.flywheelVelocity;
    }

    /** Check if flywheel is at target velocity. */
    @AutoLogOutput(key = "AtTargetVelocity")
    public boolean atTargetVelocity() {
        AngularVelocity tolerance = ShooterConstants.kFlywheelVelocityTolerance;
        return this.flywheelFailed
                || Math.abs(this.inputs.flywheelVelocity.in(RPM) - this.flywheelSetpoint.in(RPM)) < tolerance.in(RPM);
    }

    public boolean isFailed() {
        return this.flywheelFailed;
    }

    public void resetFailureFlags() {
        this.flywheelFailed = false;
    }

    public boolean isRunning() {
        return 1.0 < Math.abs(flywheelSetpoint.in(RPM));
    }

    @AutoLogOutput(key = "FlywheelSetpoint")
    public AngularVelocity getFlywheelSetpoint() {
        return this.flywheelSetpoint;
    }

    // ========== Command Factory Methods ==========

    public Command spinUpFlywheels(AngularVelocity velocity) {
        return Commands.runOnce(() -> this.setFlywheelVelocity(velocity), this).withName(this.config.name() + "SpinUp");
    }

    public Command spinUpFlywheels(@NotNull Supplier<AngularVelocity> velocitySupplier) {
        return Commands.run(() -> this.setFlywheelVelocity(velocitySupplier.get()), this)
                .withName(this.config.name() + "SpinUp");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName(this.config.name() + "Stop");
    }

    public Command waitUntilReady() {
        return Commands.waitUntil(this::atTargetVelocity).withName(this.config.name() + "WaitUntilReady");
    }

    public Command spinUpAndWait(AngularVelocity velocity) {
        return this.spinUpFlywheels(velocity).andThen(this.waitUntilReady()).withName(this.config.name() + "SpinUpAndWait");
    }
}
