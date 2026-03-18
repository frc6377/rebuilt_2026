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
public class TunablePIDController {
    /** Nominal battery voltage used to convert feedforward output (volts) to duty cycle. */
    private static final double kNominalVoltage = 12.0;

    private final String tunableName;
    private final DoubleSupplier encoderPosition;
    private final Consumer<Double> outputConsumer;

    private final LoggedNetworkNumber tunableKP;
    private final LoggedNetworkNumber tunableKI;
    private final LoggedNetworkNumber tunableKD;

    private double lastKP;
    private double lastKI;
    private double lastKD;

    private final DoubleSupplier velocitySupplier;
    private final LoggedNetworkNumber tunableKS;
    private final LoggedNetworkNumber tunableKV;
    private final LoggedNetworkNumber tunableKA;
    private double lastKS;
    private double lastKV;
    private double lastKA;
    private SimpleMotorFeedforward feedforward; // recreated when kS/kV/kA change

    private final PIDController pidController;
    private double setpoint;

    /**
     * Creates a tunable PID controller that reads position from the encoder and sends motor percentage (duty cycle) to
     * the consumer. Output is clamped to [-1, 1].
     *
     * @param tunableName Name prefix for NetworkTables entries (e.g. "Extender")
     * @param kP Initial proportional gain
     * @param kI Initial integral gain
     * @param kD Initial derivative gain
     * @param encoderPosition Supplier of current position (e.g. {@code () -> encoder.getDistance()} or {@code () ->
     *     dutyCycleEncoder.get()})
     * @param outputConsumer Receives the clamped percentage in [-1, 1] (e.g. {@code percent -> motor.set(percent)})
     */
    public TunablePIDController(
            String tunableName,
            double kP,
            double kI,
            double kD,
            DoubleSupplier encoderPosition,
            Consumer<Double> outputConsumer) {
        this.tunableName = tunableName;
        this.encoderPosition = encoderPosition;
        this.outputConsumer = outputConsumer;

        this.tunableKP = new LoggedNetworkNumber(tunableName + "/kP", kP);
        this.tunableKI = new LoggedNetworkNumber(tunableName + "/kI", kI);
        this.tunableKD = new LoggedNetworkNumber(tunableName + "/kD", kD);

        this.lastKP = kP;
        this.lastKI = kI;
        this.lastKD = kD;

        this.velocitySupplier = null;
        this.tunableKS = null;
        this.tunableKV = null;
        this.tunableKA = null;
        this.feedforward = null;

        this.pidController = new PIDController(kP, kI, kD);
        this.setpoint = 0.0;
    }

    /**
     * Creates a tunable PID controller with SimpleMotorFeedforward. Feedforward gains (kS, kV, kA) are tunable via
     * NetworkTables. Output is clamped to [-1, 1].
     *
     * @param tunableName Name prefix for NetworkTables entries
     * @param kP Initial proportional gain
     * @param kI Initial integral gain
     * @param kD Initial derivative gain
     * @param kS Static gain (feedforward)
     * @param kV Velocity gain (feedforward)
     * @param kA Acceleration gain (feedforward)
     * @param encoderPosition Supplier of current position
     * @param velocitySupplier Supplier of current velocity (for feedforward)
     * @param outputConsumer Receives the clamped percentage in [-1, 1]
     */
    public TunablePIDController(
            String tunableName,
            double kP,
            double kI,
            double kD,
            double kS,
            double kV,
            double kA,
            DoubleSupplier encoderPosition,
            DoubleSupplier velocitySupplier,
            Consumer<Double> outputConsumer) {
        this.tunableName = tunableName;
        this.encoderPosition = encoderPosition;
        this.outputConsumer = outputConsumer;
        this.velocitySupplier = velocitySupplier;

        this.tunableKP = new LoggedNetworkNumber(tunableName + "/kP", kP);
        this.tunableKI = new LoggedNetworkNumber(tunableName + "/kI", kI);
        this.tunableKD = new LoggedNetworkNumber(tunableName + "/kD", kD);
        this.tunableKS = new LoggedNetworkNumber(tunableName + "/kS", kS);
        this.tunableKV = new LoggedNetworkNumber(tunableName + "/kV", kV);
        this.tunableKA = new LoggedNetworkNumber(tunableName + "/kA", kA);

        this.lastKP = kP;
        this.lastKI = kI;
        this.lastKD = kD;
        this.lastKS = kS;
        this.lastKV = kV;
        this.lastKA = kA;

        this.feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        this.pidController = new PIDController(kP, kI, kD);
        this.setpoint = 0.0;
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

        if (feedforward != null && tunableKS != null && tunableKV != null && tunableKA != null) {
            double currentKS = tunableKS.get();
            double currentKV = tunableKV.get();
            double currentKA = tunableKA.get();
            boolean ffChanged = currentKS != lastKS || currentKV != lastKV || currentKA != lastKA;
            if (ffChanged) {
                lastKS = currentKS;
                lastKV = currentKV;
                lastKA = currentKA;
                feedforward = new SimpleMotorFeedforward(currentKS, currentKV, currentKA);
                changed = true;
            }
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
        double pidOutput = pidController.calculate(encoderPosition.getAsDouble(), setpoint);
        double ffOutput = 0.0;
        if (feedforward != null && velocitySupplier != null) {
            ffOutput = feedforward.calculate(velocitySupplier.getAsDouble()) / kNominalVoltage;
        }
        double output = MathUtil.clamp(pidOutput + ffOutput, -1.0, 1.0);
        outputConsumer.accept(output);
    }

    /** Returns the underlying WPILib PIDController (e.g. for atSetpoint(), getPositionError()). */
    public PIDController getController() {
        return pidController;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public String getTunableName() {
        return tunableName;
    }
}
