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

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.TunableTalonFX;

public class IndexerIOTalonFX implements IndexerIO {
    private final TunableTalonFX indexerMotor;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

    public IndexerIOTalonFX() {
        // Create TunableTalonFX with initial gains from IndexerConstants
        indexerMotor = new TunableTalonFX(
                Constants.CANIDs.kIndexerFlywheelOneCANID, "Drive Base", "Indexer", IndexerConstants.kIndexerGains);

        // Apply full configuration (motor output, current limits, etc.)
        TalonFXConfiguration config = IndexerConstants.kIndexerTalonFXConfiguration;
        tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(config, 0.25));

        // Signals
        position = indexerMotor.getPosition();
        velocity = indexerMotor.getVelocity();
        appliedVolts = indexerMotor.getMotorVoltage();
        current = indexerMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Update tunable gains from NetworkTables if changed
        indexerMotor.updateTunableGains();

        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

        inputs.indexerConnected = BaseStatusSignal.isAllGood(position, velocity, appliedVolts, current);
        inputs.indexerPosition = position.getValue();
        inputs.indexerVelocity = velocity.getValue();
        inputs.indexerAppliedVolts = appliedVolts.getValue();
        inputs.indexerCurrent = current.getValue();
    }

    @Override
    public void setSpeed(double speed) {
        indexerMotor.setControl(dutyCycleRequest.withOutput(speed));
    }

    @Override
    public void stop() {
        indexerMotor.setControl(new NeutralOut());
    }
}
