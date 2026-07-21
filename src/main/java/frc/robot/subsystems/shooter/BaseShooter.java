package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.energy.FinanceDepartment;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Base shooter subsystem. */
public class BaseShooter extends SubsystemBase {
    private final BaseShooterIO io;
    private final BaseShooterIOInputsAutoLogged inputs = new BaseShooterIOInputsAutoLogged();
    private final ShooterConstants.ShooterConfig config;

    private final SysIdRoutine sysIdRoutine;

    // Setpoints
    private AngularVelocity flywheelSetpoint = RPM.of(0.0);

    // Failure state
    private boolean flywheelFailed = false;

    public BaseShooter(BaseShooterIO io, ShooterConstants.ShooterConfig config) {
        this.io = io;
        this.config = config;

        sysIdRoutine = new SysIdRoutine(
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
        io.updateInputs(inputs);
        Logger.processInputs(config.name(), inputs);
        Logger.recordOutput(config.name() + "/Enabled", config.enabled());
        Logger.recordOutput(config.name() + "/FollowerEnabled", config.followerEnabled());
        Logger.recordOutput(config.name() + "/FlywheelFailed", flywheelFailed);
        Logger.recordOutput(config.name() + "/FlywheelSetpoint", flywheelSetpoint);
        if (getCurrentCommand() != null) {
            Logger.recordOutput(
                    config.name() + "/CurrentCommand", getCurrentCommand().getName());
        } else {
            Logger.recordOutput(config.name() + "/CurrentCommand", "None");
        }

        if (config.followerEnabled()) {
            FinanceDepartment.getInstance()
                    .reportCurrentUsage(
                            config.name(),
                            false,
                            inputs.flywheelCurrent.in(Amps),
                            inputs.followerFlywheelCurrent.in(Amps));
        } else {
            FinanceDepartment.getInstance().reportCurrentUsage(config.name(), false, inputs.flywheelCurrent.in(Amps));
        }
    }

    /**
     * Set flywheel velocity.
     *
     * @param velocity Target velocity
     */
    public void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelSetpoint = velocity;

        if (config.enabled() && !flywheelFailed) {
            io.setFlywheelVelocity(velocity);
        } else {
            io.setFlywheelVelocity(RPM.of(0.0));
        }
    }

    /** Stop all motors. */
    public void stop() {
        flywheelSetpoint = RPM.of(0.0);
        io.stop();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /** Get current flywheel velocity. */
    public AngularVelocity getFlywheelVelocity() {
        return inputs.flywheelVelocity;
    }

    /** Check if flywheel is at target velocity. */
    @AutoLogOutput(key = "AtTargetVelocity")
    public boolean atTargetVelocity() {
        AngularVelocity tolerance = ShooterConstants.kFlywheelVelocityTolerance;
        return flywheelFailed
                || Math.abs(inputs.flywheelVelocity.in(RPM) - flywheelSetpoint.in(RPM)) < tolerance.in(RPM);
    }

    public boolean isFailed() {
        return flywheelFailed;
    }

    public void resetFailureFlags() {
        flywheelFailed = false;
    }

    public boolean isRunning() {
        return Math.abs(flywheelSetpoint.in(RPM)) > 1.0;
    }

    @AutoLogOutput(key = "FlywheelSetpoint")
    public AngularVelocity getFlywheelSetpoint() {
        return flywheelSetpoint;
    }

    // ========== Command Factory Methods ==========

    public Command spinUpFlywheels(AngularVelocity velocity) {
        return Commands.runOnce(() -> setFlywheelVelocity(velocity), this).withName(config.name() + "SpinUp");
    }

    public Command spinUpFlywheels(Supplier<AngularVelocity> velocitySupplier) {
        return Commands.run(() -> setFlywheelVelocity(velocitySupplier.get()), this)
                .withName(config.name() + "SpinUp");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName(config.name() + "Stop");
    }

    public Command waitUntilReady() {
        return Commands.waitUntil(this::atTargetVelocity).withName(config.name() + "WaitUntilReady");
    }

    public Command spinUpAndWait(AngularVelocity velocity) {
        return spinUpFlywheels(velocity).andThen(waitUntilReady()).withName(config.name() + "SpinUpAndWait");
    }
}
