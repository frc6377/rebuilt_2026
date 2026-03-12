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
    private final String tunableName;
    private final DoubleSupplier encoderPosition;
    private final Consumer<Double> outputConsumer;

    private final LoggedNetworkNumber tunableKP;
    private final LoggedNetworkNumber tunableKI;
    private final LoggedNetworkNumber tunableKD;

    private double lastKP;
    private double lastKI;
    private double lastKD;

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
        double output = MathUtil.clamp(pidController.calculate(encoderPosition.getAsDouble(), setpoint), -1.0, 1.0);
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
