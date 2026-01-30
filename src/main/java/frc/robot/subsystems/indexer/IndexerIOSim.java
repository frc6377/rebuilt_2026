// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class IndexerIOSim implements IndexerIO {
    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getFalcon500Foc(IndexerConstants.kMotorCount),
                    IndexerConstants.kMOI.in(KilogramSquareMeters),
                    IndexerConstants.kGearRatio),
            DCMotor.getFalcon500Foc(IndexerConstants.kMotorCount),
            IndexerConstants.kGearRatio);

    private final PIDController pidController =
            new PIDController(IndexerConstants.kSimP, IndexerConstants.kSimI, IndexerConstants.kSimD);

    private double appliedVolts = 0.0;

    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        flywheelSim.update(Robot.defaultPeriodSecs);

        inputs.IndexerMotorConnected = true;
        inputs.IndexerPosition = inputs.IndexerPosition.plus(
                Radians.of(flywheelSim.getAngularVelocity().in(RadiansPerSecond) * Robot.defaultPeriodSecs));
        inputs.IndexerVelocity = flywheelSim.getAngularVelocity();
        inputs.IndexerAppliedVolts = Volts.of(appliedVolts);
        inputs.IndexerCurrent = Amps.of(flywheelSim.getCurrentDrawAmps());
    }

    @Override
    public void setVoltage(Voltage volts) {
        appliedVolts = volts.in(Volts);
        flywheelSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setVelocity(AngularVelocity velocityRadPerSec) {
        double output = pidController.calculate(
                flywheelSim.getAngularVelocity().in(RadiansPerSecond), velocityRadPerSec.in(RadiansPerSecond));
        setVoltage(Volts.of(edu.wpi.first.math.MathUtil.clamp(output, -12.0, 12.0)));
    }

    @Override
    public void stop() {
        setVoltage(Volts.of(0.0));
        pidController.reset();
    }

    @Override
    public void updatePIDConfig(Slot0Configs config) {
        // For simulation, we use the sim-specific PID values, but we could map the Slot0 values if needed
        // The tuning class also provides sim-specific getters if we want to use those
        pidController.setP(config.kP);
        pidController.setI(config.kI);
        pidController.setD(config.kD);
    }
}
