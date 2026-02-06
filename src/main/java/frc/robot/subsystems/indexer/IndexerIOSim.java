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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class IndexerIOSim implements IndexerIO {
    private static final double BATTERY_VOLTAGE = 12.0;

    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getFalcon500Foc(1),
                    IndexerConstants.kMOI.in(KilogramSquareMeters),
                    IndexerConstants.kGearRatio),
            DCMotor.getFalcon500Foc(1),
            IndexerConstants.kGearRatio);

    private double appliedVolts = 0.0;

    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        flywheelSim.update(Robot.defaultPeriodSecs);

        inputs.indexerConnected = true;
        inputs.indexerPosition = inputs.indexerPosition.plus(
                Radians.of(flywheelSim.getAngularVelocity().in(RadiansPerSecond) * Robot.defaultPeriodSecs));
        inputs.indexerVelocity = flywheelSim.getAngularVelocity();
        inputs.indexerAppliedVolts = Volts.of(appliedVolts);
        inputs.indexerCurrent = Amps.of(flywheelSim.getCurrentDrawAmps());
    }

    @Override
    public void setSpeed(double speed) {
        appliedVolts = speed * BATTERY_VOLTAGE;
        flywheelSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void stop() {
        setSpeed(0.0);
    }
}
