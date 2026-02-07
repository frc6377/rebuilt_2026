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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
    private final SingleJointedArmSim sim;
    private final PIDController controller;

    private Angle setpoint = Degrees.of(35.0);
    private Voltage appliedVoltage = Volts.of(0.0);

    public HoodIOSim() {
        // Create hood simulation (Kraken x44 motor, single jointed arm)
        // Using lower MOI and shorter arm for more responsive simulation
        sim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1), // Kraken x44 approximated with Kraken X60
                HoodConstants.gearRatio,
                0.01, // MOI (kg*m^2) - small hood mechanism
                0.15, // Arm length (m) - short hood
                HoodConstants.minAngle.in(Radians),
                HoodConstants.maxAngle.in(Radians),
                false, // Disable gravity for smoother simulation
                Degrees.of(35.0).in(Radians) // Starting angle (middle of range)
                );

        // Create PID controller with lower gains for simulation stability
        // Simulation PID is separate from real robot PID
        controller = new PIDController(0.5, 0.0, 0.02);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        // PID controller for position (simulation)
        double volts = MathUtil.clamp(
                controller.calculate(Math.toDegrees(sim.getAngleRads()), setpoint.in(Degrees)), -12.0, 12.0);
        appliedVoltage = Volts.of(volts);
        sim.setInputVoltage(volts);

        // Update simulation
        sim.update(0.02); // 20ms period

        // Update inputs
        inputs.angle = Radians.of(sim.getAngleRads());
        inputs.appliedVoltage = appliedVoltage;
        inputs.current = Amps.of(sim.getCurrentDrawAmps());
        inputs.temp = Celsius.of(25.0); // Simulated temp
    }

    @Override
    public void setAngle(Angle angle) {
        setpoint = Degrees.of(MathUtil.clamp(
                angle.in(Degrees), HoodConstants.minAngle.in(Degrees), HoodConstants.maxAngle.in(Degrees)));
    }

    @Override
    public void stop() {
        // Stop control by setting setpoint to current angle and removing voltage
        setpoint = Radians.of(sim.getAngleRads());
        appliedVoltage = Volts.of(0.0);
        sim.setInputVoltage(0.0);
    }
}
