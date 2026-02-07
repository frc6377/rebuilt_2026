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

public class UpgoerIOSim implements UpgoerIO {
    private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
    private static final double ROLLER_MASS_KG = 0.25;
    private static final double ROLLER_RADIUS_M = 0.02;
    private static final double ROLLER_MOI = 0.5 * ROLLER_MASS_KG * ROLLER_RADIUS_M * ROLLER_RADIUS_M;
    private static final double ROLLER_GEARING = 1.0;

    private final FlywheelSim sim;
    private final PIDController controller;
    private final SimpleMotorFeedforward feedforward;

    private double setpointRPM = 0.0;
    private double appliedVolts = 0.0;

    private double motorTempCelsius = 25.0;
    private static final double AMBIENT_TEMP = 25.0;
    private static final double THERMAL_RESISTANCE = 0.5;
    private static final double THERMAL_TIME_CONSTANT = 30.0;

    public UpgoerIOSim() {
        var plant = LinearSystemId.createFlywheelSystem(MOTOR, ROLLER_MOI, ROLLER_GEARING);
        sim = new FlywheelSim(plant, MOTOR);

        controller = new PIDController(0.001, 0.0005, 0.0);

        double freeSpeedRPM = MOTOR.freeSpeedRadPerSec * 60.0 / (2.0 * Math.PI);
        double kv = 12.0 / freeSpeedRPM;
        feedforward = new SimpleMotorFeedforward(0.0, kv, 0.0);
    }

    @Override
    public void updateInputs(UpgoerIOInputs inputs) {
        double ff = feedforward.calculate(setpointRPM);
        double fb = controller.calculate(sim.getAngularVelocityRPM(), setpointRPM);
        appliedVolts = MathUtil.clamp(ff + fb, -12.0, 12.0);

        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);

        double powerDissipated = sim.getCurrentDrawAmps() * Math.abs(appliedVolts) * 0.1;
        double equilibriumTemp = AMBIENT_TEMP + powerDissipated * THERMAL_RESISTANCE;
        double dt = Robot.defaultPeriodSecs;
        motorTempCelsius += (equilibriumTemp - motorTempCelsius) * dt / THERMAL_TIME_CONSTANT;

        inputs.velocity = RPM.of(sim.getAngularVelocityRPM());
        inputs.appliedVoltage = Volts.of(appliedVolts);
        inputs.current = Amps.of(sim.getCurrentDrawAmps());
        inputs.temp = Celsius.of(motorTempCelsius);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        setpointRPM = velocity.in(RPM);
    }

    @Override
    public void stop() {
        setpointRPM = 0.0;
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
    }
}
