// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * A WPILib PIDController with tunable P, I, D gains via NetworkTables. Uses a passed-in encoder (position supplier) and
 * applies the PID output to a motor via a consumer as duty cycle (percentage, -1 to 1).
 *
 * <p>This class handles PID control only. Feedforward should be implemented separately in the subsystem to allow
 * model-specific handling (e.g., ArmFeedforward with radian conversions, ElevatorFeedforward without position).
 *
 * <p>Initial gains are zero until you {@link #applyPreset}, tune the main {@code tunableName/kP} (etc.) keys from the
 * dashboard, or call {@link #updateTunableGains} after those values change.
 *
 * <p>Usage example:
 *
 * <pre>{@code
 * TunablePIDFController extenderPid = new TunablePIDFController(
 *         "Extender",
 *         () -> extenderEncoder.getPosition(),
 *         percent -> extenderMotor.set(percent));
 *
 * extenderPid.addPreset("default", new PIDConfig(0.01, 0, 0));
 * extenderPid.applyPreset("default");
 *
 * // In periodic():
 * extenderPid.updateTunableGains();
 * extenderPid.setSetpoint(targetPosition);
 * double pidOut = extenderPid.calculate();
 * double ffOut = armFF.calculate(angleRadians, velocityRadPerSec);
 * outputConsumer.accept(MathUtil.clamp(pidOut + ffOut / 12.0, -1, 1));
 * }</pre>
 */
public class TunablePIDController {
    /** Compact holder for PID gains. */
    public record PIDConfig(double kP, double kI, double kD) {}

    public static final PIDConfig defaultConfig = new PIDConfig(0.0, 0.0, 0.0);

    private final @NotNull LoggedNetworkNumber loggedOutput;

    private final String tunableName;
    private final DoubleSupplier encoderPosition;
    private final Consumer<Double> outputConsumer;

    private final @NotNull ProfiledPIDController pidController;
    private double setpoint;

    // Preset configs that can be swapped at runtime. Each preset is created on
    // demand and its values are
    // exposed via NetworkTables when added.
    private static class PresetHolder {
        final PIDConfig config;
        final @NotNull LoggedNetworkNumber kp;
        final @NotNull LoggedNetworkNumber ki;
        final @NotNull LoggedNetworkNumber kd;

        PresetHolder(String basePath, @NotNull PIDConfig cfg) {
            this.config = cfg;
            this.kp = new LoggedNetworkNumber(basePath + "/kP", cfg.kP());
            this.ki = new LoggedNetworkNumber(basePath + "/kI", cfg.kI());
            this.kd = new LoggedNetworkNumber(basePath + "/kD", cfg.kD());
        }
    }

    // Map of preset name -> holder. Starts empty (no presets logged). When a preset
    // is added we create its
    // LoggedNetworkNumber entries so it appears in NetworkTables.
    private final Map<String, PresetHolder> presets = new HashMap<>();
    private @Nullable String activePresetName = null;

    // Logged index of the active preset (for dashboard visibility).
    private final @NotNull LoggedNetworkString activePreset;

    // Last-seen values for quick change detection when reading the active preset's
    // NT entries
    private double lastKP;
    private double lastKI;
    private double lastKD;

    private final @NotNull LoggedNetworkNumber maxVelocityLog;
    private final @NotNull LoggedNetworkNumber maxAccelerationLog;
    private double lastMaxVelocity;
    private double lastMaxAcceleration;

    /**
     * Creates a new TunablePIDFController with the specified configuration, using unconstrained profiled PID control.
     *
     * @param tunableName The name of this controller, used as the NetworkTables key prefix (e.g., "Extender"). This
     *     name appears in the dashboard under LiveWindow and can be used to organize multiple controllers.
     * @param encoderPosition A DoubleSupplier providing the current position from the encoder. Units depend on your
     *     mechanism (rotations, meters, radians, etc.).
     * @param outputConsumer A Consumer that receives the computed motor output as a duty cycle percentage in the range
     *     [-1.0, 1.0]. Typically this is {@code motorController::set}.
     */
    public TunablePIDController(String tunableName, DoubleSupplier encoderPosition, Consumer<Double> outputConsumer) {
        this(
                tunableName,
                encoderPosition,
                outputConsumer,
                new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
    }

    /**
     * Creates a new TunablePIDFController with the specified configuration.
     *
     * @param tunableName The name of this controller, used as the NetworkTables key prefix (e.g., "Extender"). This
     *     name appears in the dashboard under LiveWindow and can be used to organize multiple controllers.
     * @param encoderPosition A DoubleSupplier providing the current position from the encoder. Units depend on your
     *     mechanism (rotations, meters, radians, etc.).
     * @param outputConsumer A Consumer that receives the computed motor output as a duty cycle percentage in the range
     *     [-1.0, 1.0]. Typically this is {@code motorController::set}.
     * @param constraints The profile constraints.
     */
    public TunablePIDController(
            String tunableName,
            DoubleSupplier encoderPosition,
            Consumer<Double> outputConsumer,
            TrapezoidProfile.@NotNull Constraints constraints) {
        this.tunableName = tunableName;
        this.encoderPosition = encoderPosition;
        this.outputConsumer = outputConsumer;

        this.pidController =
                new ProfiledPIDController(defaultConfig.kP(), defaultConfig.kI(), defaultConfig.kD(), constraints);
        this.setpoint = 0.0;

        this.activePreset = new LoggedNetworkString(tunableName + "/ActivePreset", "None");

        this.maxVelocityLog = new LoggedNetworkNumber(tunableName + "/MaxVelocity", constraints.maxVelocity);
        this.maxAccelerationLog =
                new LoggedNetworkNumber(tunableName + "/MaxAcceleration", constraints.maxAcceleration);
        this.loggedOutput = new LoggedNetworkNumber(tunableName + "AppliedOutput", 0.0);

        // Initialize last-seen values to defaults so first read will be detected as a
        // change
        this.lastKP = defaultConfig.kP();
        this.lastKI = defaultConfig.kI();
        this.lastKD = defaultConfig.kD();
        this.lastMaxVelocity = constraints.maxVelocity;
        this.lastMaxAcceleration = constraints.maxAcceleration;
    }

    /**
     * Updates the profiled PID speed constraints at runtime.
     *
     * @param maxVelocity Maximum profile velocity.
     * @param maxAcceleration Maximum profile acceleration.
     */
    public void setSpeedConstraints(double maxVelocity, double maxAcceleration) {
        this.setSpeedConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }

    /**
     * Updates the profiled PID speed constraints at runtime.
     *
     * @param constraints New trapezoid profile constraints.
     */
    public void setSpeedConstraints(TrapezoidProfile.@Nullable Constraints constraints) {
        if (null == constraints || constraints == this.pidController.getConstraints()) {
            return;
        }
        this.pidController.setConstraints(constraints);
        this.maxVelocityLog.set(constraints.maxVelocity);
        this.maxAccelerationLog.set(constraints.maxAcceleration);
        this.lastMaxVelocity = constraints.maxVelocity;
        this.lastMaxAcceleration = constraints.maxAcceleration;
    }

    /** Backward-compatible typo alias. Prefer {@link #setSpeedConstraints(double, double)}. */
    public void setSpeedConstrants(double maxVelocity, double maxAcceleration) {
        this.setSpeedConstraints(maxVelocity, maxAcceleration);
    }

    /**
     * Adds or replaces a named preset. Does not apply it automatically.
     *
     * @param name The name of the preset.
     * @param config The PID gains for this preset.
     */
    public void addPreset(@Nullable String name, @Nullable PIDConfig config) {
        if (null == name || null == config) {
            return;
        }
        // Create networktable-backed entries for this preset so it appears in the
        // dashboard
        PresetHolder holder = new PresetHolder(this.tunableName + "/Presets/" + name, config);
        this.presets.put(name, holder);
    }

    /**
     * Removes a preset by name.
     *
     * @param name The name of the preset to remove.
     * @return true if the preset was found and removed, false otherwise.
     */
    public boolean removePreset(@Nullable String name) {
        if (null == name) {
            return false;
        }
        PresetHolder removedHolder = this.presets.remove(name);
        boolean removed = null != removedHolder;
        if (removed && name.equals(this.activePresetName)) {
            this.activePresetName = null;
        }
        return removed;
    }

    /** Returns an immutable list of available preset names. */
    public @NotNull List<String> getPresetNames() {
        List<String> names = new ArrayList<>(this.presets.keySet());
        Collections.sort(names);
        return Collections.unmodifiableList(names);
    }

    /** Returns the currently active preset name, or null if none. */
    public String getActivePresetName() {
        return this.activePresetName;
    }

    /**
     * Applies a preset by name. Immediately updates the PID controller with the preset's gains. NetworkTables entries
     * for the preset are synchronized.
     *
     * @param name The name of the preset to apply.
     * @return true if the preset was found and applied, false otherwise.
     */
    public boolean applyPreset(String name) {
        PresetHolder holder = this.presets.get(name);
        if (null == holder || name == this.activePresetName) {
            return false;
        }

        PIDConfig cfg = holder.config;

        // Ensure the preset's NT entries reflect the config we just applied
        holder.kp.set(cfg.kP());
        holder.ki.set(cfg.kI());
        holder.kd.set(cfg.kD());

        // Update controller internals immediately
        this.pidController.setP(cfg.kP());
        this.pidController.setI(cfg.kI());
        this.pidController.setD(cfg.kD());

        // Update last-seen cache so subsequent updateTunableGains reads don't re-apply
        this.lastKP = cfg.kP();
        this.lastKI = cfg.kI();
        this.lastKD = cfg.kD();

        // Set active preset
        this.activePresetName = name;
        this.activePreset.set(name);
        return true;
    }

    /**
     * Call this periodically (e.g. in subsystem periodic()) to apply any PID gain changes from the dashboard to the
     * internal PIDController.
     *
     * @return true if any gains were updated, false otherwise.
     */
    public boolean updateTunableGains() {
        boolean changed = false;

        double currentMaxV = this.maxVelocityLog.get();
        double currentMaxA = this.maxAccelerationLog.get();

        if (currentMaxV != this.lastMaxVelocity || currentMaxA != this.lastMaxAcceleration) {
            this.lastMaxVelocity = currentMaxV;
            this.lastMaxAcceleration = currentMaxA;
            this.pidController.setConstraints(new TrapezoidProfile.Constraints(currentMaxV, currentMaxA));
            changed = true;
        }

        if (null == this.activePresetName) {
            return changed;
        }

        PresetHolder holder = this.presets.get(this.activePresetName);
        if (null == holder) {
            return changed;
        }

        double currentKP = holder.kp.get();
        double currentKI = holder.ki.get();
        double currentKD = holder.kd.get();

        boolean pidChanged = currentKP != this.lastKP || currentKI != this.lastKI || currentKD != this.lastKD;

        if (pidChanged) {
            this.lastKP = currentKP;
            this.lastKI = currentKI;
            this.lastKD = currentKD;
            this.pidController.setP(currentKP);
            this.pidController.setI(currentKI);
            this.pidController.setD(currentKD);
        }

        return changed || pidChanged;
    }

    /**
     * Sets the PID setpoint (goal position).
     *
     * @param setpoint The desired position.
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Calculates the PID output based on the current position and setpoint. Does not apply the output to the motor; the
     * caller is responsible for combining with feedforward (if needed) and sending to the motor.
     *
     * @return The PID output in the range [-1.0, 1.0].
     */
    public double calculate() {
        double pidOut = this.pidController.calculate(this.encoderPosition.getAsDouble(), this.setpoint);

        return MathUtil.clamp(pidOut, -1.0, 1.0);
    }

    /**
     * Calculates the PID output and immediately applies it to the motor via the output consumer. Use this if you don't
     * have feedforward. If you need feedforward, use {@link #calculate()} instead and combine with your feedforward
     * calculation before sending to the motor.
     */
    public void runPid() {
        double output = this.calculate();
        this.loggedOutput.set(output);
        this.outputConsumer.accept(output);
    }

    /** Returns the underlying WPILib ProfiledPIDController (e.g. for atSetpoint(), getPositionError()). */
    public ProfiledPIDController getPIDController() {
        return this.pidController;
    }

    /** Returns the current PID setpoint. */
    public double getSetpoint() {
        return this.setpoint;
    }

    /** Returns the tunable name of this controller. */
    public String getTunableName() {
        return this.tunableName;
    }
}
