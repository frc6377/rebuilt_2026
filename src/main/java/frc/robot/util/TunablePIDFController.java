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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.Map;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


/**
 * A WPILib PIDController with tunable P, I, D gains via NetworkTables. Uses a passed-in encoder (position supplier) and
 * applies the PID output to a motor via a consumer as duty cycle (percentage, -1 to 1).
 *
 * <p>Usage example:
 *
 * <pre>{@code
 * TunablePIDController extenderPid = new TunablePIDController(
 *     "Extender",
 *     0.1, 0.0, 0.01,
 *     () -> extenderEncoder.get(),
 *     percent -> extenderMotor.set(percent));
 *
 * // In periodic():
 * extenderPid.updateTunableGains();
 * extenderPid.setSetpoint(targetPosition);
 * extenderPid.runPid();
 * }</pre>
 */
public class TunablePIDFController {
    /**
     * Compact holder for PID + feedforward gains.
     */
    public static record PIDFConfig(
        double kP, double kI, double kD, double kS, double kV, double kA) {}
        
    public static final PIDFConfig defaultConfig = new PIDFConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    private final String tunableName;
    private final DoubleSupplier encoderPosition;
    private final Consumer<Double> outputConsumer;

    private final LoggedNetworkNumber tunableKP;
    private final LoggedNetworkNumber tunableKI;
    private final LoggedNetworkNumber tunableKD;


    // Optional feedforward tunables and state
    private LoggedNetworkNumber tunableKS;
    private LoggedNetworkNumber tunableKV;
    private LoggedNetworkNumber tunableKA;

    private double lastKP;
    private double lastKI;
    private double lastKD;

    private double lastKS;
    private double lastKV;
    private double lastKA;

    private final PIDController pidController;
    private double setpoint;

    // Optional feedforward and velocity supplier (null when not used)
    private SimpleMotorFeedforward feedforward;
    private DoubleSupplier velocitySupplier;

    // Preset configs that can be swapped at runtime. Each preset is created on demand and its values are
    // exposed via NetworkTables when added.
    private static class PresetHolder {
        final PIDFConfig config;
        final LoggedNetworkNumber kp;
        final LoggedNetworkNumber ki;
        final LoggedNetworkNumber kd;
        final LoggedNetworkNumber ks;
        final LoggedNetworkNumber kv;
        final LoggedNetworkNumber ka;

        PresetHolder(String basePath, PIDFConfig cfg) {
            this.config = cfg;
            this.kp = new LoggedNetworkNumber(basePath + "/kP", cfg.kP());
            this.ki = new LoggedNetworkNumber(basePath + "/kI", cfg.kI());
            this.kd = new LoggedNetworkNumber(basePath + "/kD", cfg.kD());
            this.ks = new LoggedNetworkNumber(basePath + "/kS", cfg.kS());
            this.kv = new LoggedNetworkNumber(basePath + "/kV", cfg.kV());
            this.ka = new LoggedNetworkNumber(basePath + "/kA", cfg.kA());
        }
    }

    // Map of preset name -> holder. Starts empty (no presets logged). When a preset is added we create its
    // LoggedNetworkNumber entries so it appears in NetworkTables.
    private final Map<String, PresetHolder> presets = new HashMap<>();
    private String activePresetName = null;

    

    

    /**
     * Constructor that accepts a combined PID + feedforward config. If you don't want feedforward, pass zeros
     * for kS/kV/kA and a null velocitySupplier.
     */
    public TunablePIDFController(
            String tunableName,
            DoubleSupplier encoderPosition,
            Consumer<Double> outputConsumer,
            DoubleSupplier velocitySupplier) {
        this.tunableName = tunableName;
        this.encoderPosition = encoderPosition;
        this.outputConsumer = outputConsumer;

        this.tunableKP = new LoggedNetworkNumber(tunableName + "/kP", defaultConfig.kP());
        this.tunableKI = new LoggedNetworkNumber(tunableName + "/kI", defaultConfig.kI());
        this.tunableKD = new LoggedNetworkNumber(tunableName + "/kD", defaultConfig.kD());

        this.lastKP = defaultConfig.kP();
        this.lastKI = defaultConfig.kI();
        this.lastKD = defaultConfig.kD();

        // Feedforward tunables
        this.tunableKS = new LoggedNetworkNumber(tunableName + "/kS", defaultConfig.kS());
        this.tunableKV = new LoggedNetworkNumber(tunableName + "/kV", defaultConfig.kV());
        this.tunableKA = new LoggedNetworkNumber(tunableName + "/kA", defaultConfig.kA());

        this.lastKS = defaultConfig.kS();
        this.lastKV = defaultConfig.kV();
        this.lastKA = defaultConfig.kA();

        this.pidController = new PIDController(defaultConfig.kP(), defaultConfig.kI(), defaultConfig.kD());
        this.setpoint = 0.0;

        this.feedforward = new SimpleMotorFeedforward(defaultConfig.kS(), defaultConfig.kV(), defaultConfig.kA());
        this.velocitySupplier = velocitySupplier;
    }

    /** Adds or replaces a named preset. Does not apply it automatically. */
    public void addPreset(String name, PIDFConfig config) {
        if (name == null || config == null) {
            return;
        }
        // Create networktable-backed entries for this preset so it appears in the dashboard
        PresetHolder holder = new PresetHolder(tunableName + "/Presets/" + name, config);
        presets.put(name, holder);
    }

    /** Removes a preset by name. Returns true if removed. */
    public boolean removePreset(String name) {
        if (name == null) {
            return false;
        }
        PresetHolder removedHolder = presets.remove(name);
        boolean removed = removedHolder != null;
        if (removed && name.equals(activePresetName)) {
            activePresetName = null;
        }
        return removed;
    }

    /** Returns an immutable list of available preset names. */
    public List<String> getPresetNames() {
        List<String> names = new ArrayList<>(presets.keySet());
        Collections.sort(names);
        return Collections.unmodifiableList(names);
    }

    /** Returns the currently active preset name, or null if none. */
    public String getActivePresetName() {
        return activePresetName;
    }

    /** Applies a preset by name. Returns true if applied. */
    public boolean applyPreset(String name) {
        PresetHolder holder = presets.get(name);
        if (holder == null) {
            return false;
        }

        PIDFConfig cfg = holder.config;

        // Update networktable-visible tunables
        tunableKP.set(cfg.kP());
        tunableKI.set(cfg.kI());
        tunableKD.set(cfg.kD());

        // Also update the preset's own LoggedNetworkNumber entries so they reflect any changes
        holder.kp.set(cfg.kP());
        holder.ki.set(cfg.kI());
        holder.kd.set(cfg.kD());

        if (tunableKS != null) {
            tunableKS.set(cfg.kS());
            tunableKV.set(cfg.kV());
            tunableKA.set(cfg.kA());
        }

        holder.ks.set(cfg.kS());
        holder.kv.set(cfg.kV());
        holder.ka.set(cfg.kA());

        // Update controller internals
        pidController.setP(cfg.kP());
        pidController.setI(cfg.kI());
        pidController.setD(cfg.kD());

        feedforward = new SimpleMotorFeedforward(cfg.kS(), cfg.kV(), cfg.kA());

        lastKP = cfg.kP();
        lastKI = cfg.kI();
        lastKD = cfg.kD();
        lastKS = cfg.kS();
        lastKV = cfg.kV();
        lastKA = cfg.kA();

        activePresetName = name;
        return true;
    }

    /** Cycles to the next preset in alphabetical order and applies it. Returns the name applied or null if none. */
    public String cyclePreset() {
        List<String> names = getPresetNames();
        if (names.isEmpty()) {
            return null;
        }
        if (activePresetName == null) {
            applyPreset(names.get(0));
            return activePresetName;
        }
        int idx = names.indexOf(activePresetName);
        int next = (idx + 1) % names.size();
        applyPreset(names.get(next));
        return activePresetName;
    }

    /** Set or update the velocity supplier used by feedforward. */
    public void setVelocitySupplier(DoubleSupplier velocitySupplier) {
        this.velocitySupplier = velocitySupplier;
    }

    /**
     * Call this periodically (e.g. in subsystem periodic()) to apply any PID gain changes from the dashboard to the
     * internal PIDController.
     *
     * @return true if gains were updated, false otherwise
     */
    public boolean updateTunableGains() {
        double currentKP = tunableKP.get();
        double currentKI = tunableKI.get();
        double currentKD = tunableKD.get();

        boolean changed = currentKP != lastKP || currentKI != lastKI || currentKD != lastKD;

        if (changed) {
            lastKP = currentKP;
            lastKI = currentKI;
            lastKD = currentKD;
            pidController.setP(currentKP);
            pidController.setI(currentKI);
            pidController.setD(currentKD);
        }

        // Update feedforward tunables if present
        if (tunableKS != null) {
            double currentKS = tunableKS.get();
            double currentKV = tunableKV.get();
            double currentKA = tunableKA.get();

            boolean ffChanged = currentKS != lastKS || currentKV != lastKV || currentKA != lastKA;
            if (ffChanged) {
                lastKS = currentKS;
                lastKV = currentKV;
                lastKA = currentKA;
                feedforward = new SimpleMotorFeedforward(currentKS, currentKV, currentKA);
            }

            changed = changed || ffChanged;
        }

        return changed;
    }

    /** Sets the PID setpoint (goal position). */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Runs the PID loop: reads current position from the encoder, computes output, clamps to [-1, 1] (motor
     * percentage), and passes the result to the output consumer. Call this periodically (e.g. in subsystem periodic()).
     */
    public void runPid() {
        double pidOut = pidController.calculate(encoderPosition.getAsDouble(), setpoint);

        double ffPercent = 0.0;
        if (feedforward != null && velocitySupplier != null) {
            // feedforward returns volts; convert to motor percent by dividing by nominal 12V
            double ffVolts = feedforward.calculate(velocitySupplier.getAsDouble());
            ffPercent = ffVolts / 12.0;
        }

        double output = MathUtil.clamp(pidOut + ffPercent, -1.0, 1.0);
        outputConsumer.accept(output);
    }

    /** Returns the underlying WPILib PIDController (e.g. for atSetpoint(), getPositionError()). */
    public PIDController getController() {
        return pidController;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public String getTunableName() {
        return tunableName;
    }
}
