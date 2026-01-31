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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    // Motor model - Kraken X60 FOC for better performance
    private static final DCMotor FLYWHEEL_MOTOR = DCMotor.getKrakenX60Foc(1);

    // Flywheel physical properties
    // MOI = 0.5 * m * r^2 for solid cylinder
    // Assuming ~0.5kg flywheel with 2" (0.0508m) radius
    private static final double FLYWHEEL_MASS_KG = 0.5;
    private static final double FLYWHEEL_RADIUS_M = ShooterConstants.flywheelRadius.in(Meters);
    private static final double FLYWHEEL_MOI = 0.5 * FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M;
    private static final double FLYWHEEL_GEARING = 1.0; // Direct drive

    // Simulation models
    private final FlywheelSim leftFlywheelSim;
    private final FlywheelSim rightFlywheelSim;

    // PID Controllers for simulation (velocity control in RPM)
    private final PIDController leftFlywheelController;
    private final PIDController rightFlywheelController;

    // Feedforward for better velocity tracking
    private final SimpleMotorFeedforward feedforward;

    // Setpoints
    private double leftFlywheelSetpointRPM = 0.0;
    private double rightFlywheelSetpointRPM = 0.0;

    // Applied voltages
    private double leftFlywheelAppliedVolts = 0.0;
    private double rightFlywheelAppliedVolts = 0.0;

    // Temperature simulation (simple thermal model)
    private double leftMotorTempCelsius = 25.0;
    private double rightMotorTempCelsius = 25.0;
    private static final double AMBIENT_TEMP = 25.0;
    private static final double THERMAL_RESISTANCE = 0.5; // °C/W - how fast motor heats up
    private static final double THERMAL_TIME_CONSTANT = 30.0; // seconds - thermal mass

    public ShooterIOSim() {
        // Create flywheel plant using proper physics
        var flywheelPlant = LinearSystemId.createFlywheelSystem(FLYWHEEL_MOTOR, FLYWHEEL_MOI, FLYWHEEL_GEARING);

        leftFlywheelSim = new FlywheelSim(flywheelPlant, FLYWHEEL_MOTOR);
        rightFlywheelSim = new FlywheelSim(flywheelPlant, FLYWHEEL_MOTOR);

        // PID controllers tuned for simulation
        // P gain for fast response, I gain to eliminate steady-state error
        leftFlywheelController = new PIDController(0.001, 0.0005, 0.0);
        rightFlywheelController = new PIDController(0.001, 0.0005, 0.0);

        // Feedforward based on actual motor model
        // kV = Voltage / (rad/s) from motor model, converted to V/RPM
        // For Kraken X60 FOC: kV_motor = 12V / (freeSpeedRadPerSec)
        // freeSpeedRadPerSec = 6000 RPM * 2*pi/60 = 628.3 rad/s
        // kV in V/RPM = 12 / 6000 = 0.002, but we need to account for MOI
        // Using motor nominal voltage / free speed for feedforward
        double freeSpeedRPM = FLYWHEEL_MOTOR.freeSpeedRadPerSec * 60.0 / (2.0 * Math.PI);
        double kV = 12.0 / freeSpeedRPM; // Volts per RPM

        feedforward = new SimpleMotorFeedforward(
                0.0, // kS - no static friction in sim
                kV, // kV calculated from motor model
                0.0); // kA
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Calculate control output using feedforward + feedback
        double leftFF = feedforward.calculate(leftFlywheelSetpointRPM);
        double leftFB =
                leftFlywheelController.calculate(leftFlywheelSim.getAngularVelocityRPM(), leftFlywheelSetpointRPM);
        leftFlywheelAppliedVolts = MathUtil.clamp(leftFF + leftFB, -12.0, 12.0);

        double rightFF = feedforward.calculate(rightFlywheelSetpointRPM);
        double rightFB =
                rightFlywheelController.calculate(rightFlywheelSim.getAngularVelocityRPM(), rightFlywheelSetpointRPM);
        rightFlywheelAppliedVolts = MathUtil.clamp(rightFF + rightFB, -12.0, 12.0);

        // Apply voltage to simulations
        leftFlywheelSim.setInputVoltage(leftFlywheelAppliedVolts);
        rightFlywheelSim.setInputVoltage(rightFlywheelAppliedVolts);

        // Update flywheel simulations
        leftFlywheelSim.update(0.02); // 20ms period
        rightFlywheelSim.update(0.02);

        // Update temperature simulation (simple first-order thermal model)
        // Power dissipated = I^2 * R (approximated by current * voltage losses)
        double leftPowerDissipated =
                leftFlywheelSim.getCurrentDrawAmps() * Math.abs(leftFlywheelAppliedVolts) * 0.1; // 10% losses
        double rightPowerDissipated = rightFlywheelSim.getCurrentDrawAmps() * Math.abs(rightFlywheelAppliedVolts) * 0.1;

        // Temperature rises toward equilibrium based on power and thermal resistance
        double leftEquilibriumTemp = AMBIENT_TEMP + leftPowerDissipated * THERMAL_RESISTANCE;
        double rightEquilibriumTemp = AMBIENT_TEMP + rightPowerDissipated * THERMAL_RESISTANCE;

        // First-order thermal response: T_new = T_old + (T_eq - T_old) * dt / tau
        double dt = 0.02;
        leftMotorTempCelsius += (leftEquilibriumTemp - leftMotorTempCelsius) * dt / THERMAL_TIME_CONSTANT;
        rightMotorTempCelsius += (rightEquilibriumTemp - rightMotorTempCelsius) * dt / THERMAL_TIME_CONSTANT;

        // Update inputs
        inputs.leftFlywheelVelocityRPM = leftFlywheelSim.getAngularVelocityRPM();
        inputs.leftFlywheelAppliedVolts = leftFlywheelAppliedVolts;
        inputs.leftFlywheelCurrentAmps = leftFlywheelSim.getCurrentDrawAmps();
        inputs.leftFlywheelTempCelsius = leftMotorTempCelsius;

        inputs.rightFlywheelVelocityRPM = rightFlywheelSim.getAngularVelocityRPM();
        inputs.rightFlywheelAppliedVolts = rightFlywheelAppliedVolts;
        inputs.rightFlywheelCurrentAmps = rightFlywheelSim.getCurrentDrawAmps();
        inputs.rightFlywheelTempCelsius = rightMotorTempCelsius;
    }

    @Override
    public void setLeftFlywheelVelocity(AngularVelocity velocity) {
        leftFlywheelSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void setRightFlywheelVelocity(AngularVelocity velocity) {
        rightFlywheelSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void stop() {
        leftFlywheelSetpointRPM = 0.0;
        rightFlywheelSetpointRPM = 0.0;
        leftFlywheelAppliedVolts = 0.0;
        rightFlywheelAppliedVolts = 0.0;
        leftFlywheelSim.setInputVoltage(0.0);
        rightFlywheelSim.setInputVoltage(0.0);
    }

    /**
     * Get the current launch velocity based on flywheel surface speed. v_launch = omega * r * efficiency
     *
     * @return Launch velocity in meters per second
     */
    public double getLaunchVelocityMPS() {
        double avgRPM = (leftFlywheelSim.getAngularVelocityRPM() + rightFlywheelSim.getAngularVelocityRPM()) / 2.0;
        double omegaRadPerSec = avgRPM * 2.0 * Math.PI / 60.0;
        return omegaRadPerSec * FLYWHEEL_RADIUS_M * ShooterConstants.launchEfficiency;
    }
}
