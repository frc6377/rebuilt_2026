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

package frc.robot.subsystems.upgoer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import org.jetbrains.annotations.NotNull;

public class UpgoerIOSim implements UpgoerIO {
    private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
    private static final double ROLLER_MASS_KG = 0.25;
    private static final double ROLLER_RADIUS_M = 0.02;
    private static final double ROLLER_MOI = 0.5 * ROLLER_MASS_KG * ROLLER_RADIUS_M * ROLLER_RADIUS_M;
    private static final double ROLLER_GEARING = 1.0;

    private final @NotNull FlywheelSim sim;
    private final @NotNull PIDController controller;
    private final @NotNull SimpleMotorFeedforward feedforward;

    private double setpointRPM = 0.0;
    private double appliedVolts = 0.0;

    private double motorTempCelsius = 25.0;
    private static final double AMBIENT_TEMP = 25.0;
    private static final double THERMAL_RESISTANCE = 0.5;
    private static final double THERMAL_TIME_CONSTANT = 30.0;

    /**
     * @param motorId CAN ID (unused in sim, kept for API consistency).
     * @param logName Logging key prefix (unused in sim, kept for API consistency).
     */
    @SuppressWarnings("unused")
    public UpgoerIOSim(int motorId, String logName) {
        var plant = LinearSystemId.createFlywheelSystem(MOTOR, ROLLER_MOI, ROLLER_GEARING);
        this.sim = new FlywheelSim(plant, MOTOR);

        this.controller = new PIDController(0.001, 0.0005, 0.0);

        double freeSpeedRPM = MOTOR.freeSpeedRadPerSec * 60.0 / (2.0 * Math.PI);
        double kv = 12.0 / freeSpeedRPM;
        this.feedforward = new SimpleMotorFeedforward(0.0, kv, 0.0);
    }

    @Override
    public void updateInputs(@NotNull UpgoerIOInputs inputs) {
        double ff = this.feedforward.calculate(this.setpointRPM);
        double fb = this.controller.calculate(this.sim.getAngularVelocityRPM(), this.setpointRPM);
        this.appliedVolts = MathUtil.clamp(ff + fb, -12.0, 12.0);

        this.sim.setInputVoltage(this.appliedVolts);
        this.sim.update(0.02);

        double powerDissipated = this.sim.getCurrentDrawAmps() * Math.abs(this.appliedVolts) * 0.1;
        double equilibriumTemp = AMBIENT_TEMP + powerDissipated * THERMAL_RESISTANCE;
        double dt = Robot.defaultPeriodSecs;
        this.motorTempCelsius += (equilibriumTemp - this.motorTempCelsius) * dt / THERMAL_TIME_CONSTANT;

        inputs.velocity = RPM.of(this.sim.getAngularVelocityRPM());
        inputs.appliedVoltage = Volts.of(this.appliedVolts);
        inputs.statorCurrent = Amps.of(this.sim.getCurrentDrawAmps());
        inputs.supplyCurrent = Amps.of(this.sim.getCurrentDrawAmps());
        inputs.temp = Celsius.of(this.motorTempCelsius);
        inputs.velocityError = RPM.of(this.setpointRPM - this.sim.getAngularVelocityRPM());
    }

    @Override
    public void setVelocity(@NotNull AngularVelocity velocity) {
        this.setpointRPM = velocity.in(RPM);
    }

    @Override
    public void stop() {
        this.setpointRPM = 0.0;
        this.appliedVolts = 0.0;
        this.sim.setInputVoltage(0.0);
    }
}
