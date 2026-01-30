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

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
    private final SingleJointedArmSim sim;
    private final PIDController controller;

    private double setpointDegrees = 0.0;
    private double appliedVolts = 0.0;

    public HoodIOSim() {
        // Create hood simulation (Kraken x44 motor, single jointed arm)
        sim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1), // Kraken x44 approximated with Kraken X60
                HoodConstants.gearRatio,
                0.1, // MOI (kg*m^2) - adjust based on actual hood mass
                0.3, // Arm length (m) - adjust based on actual hood geometry
                HoodConstants.minAngle.in(Radians),
                HoodConstants.maxAngle.in(Radians),
                true, // Simulate gravity
                HoodConstants.minAngle.in(Radians) // Starting angle
                );

        // Create PID controller
        controller = new PIDController(HoodConstants.defaultKP, HoodConstants.defaultKI, HoodConstants.defaultKD);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        // PID controller for position (simulation)
        appliedVolts =
                MathUtil.clamp(controller.calculate(Math.toDegrees(sim.getAngleRads()), setpointDegrees), -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);

        // Update simulation
        sim.update(0.02); // 20ms period

        // Update inputs
        inputs.angleDegrees = Math.toDegrees(sim.getAngleRads());
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.tempCelsius = 25.0; // Simulated temp
    }

    @Override
    public void setAngle(Angle angle) {
        setpointDegrees = MathUtil.clamp(
                angle.in(Degrees), HoodConstants.minAngle.in(Degrees), HoodConstants.maxAngle.in(Degrees));
    }

    @Override
    public void stop() {
        // Stop control by setting setpoint to current angle and removing voltage
        setpointDegrees = Math.toDegrees(sim.getAngleRads());
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
    }
}
