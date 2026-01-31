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

package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.TunableTalonFX;

public class ShooterIOTalonFX implements ShooterIO {
    private final TunableTalonFX shooterMotor;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    public ShooterIOTalonFX() {
        // Create TunableTalonFX with initial gains from ShooterConstants
        shooterMotor = new TunableTalonFX(
                Constants.CANIDs.kShooterFlywheelOneCANID,
                "Drive Base",
                "Shooter/ShooterMotor");

        // Apply full configuration (motor output, current limits, etc.)
        TalonFXConfiguration config = ShooterConstants.kShooterTalonFXConfiguration;
        tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(config, 0.25));

        // Signals
        position = shooterMotor.getPosition();
        velocity = shooterMotor.getVelocity();
        appliedVolts = shooterMotor.getMotorVoltage();
        current = shooterMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update tunable gains from NetworkTables if changed
        shooterMotor.updateTunableGains();

        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

        inputs.shooterMotorConnected = BaseStatusSignal.isAllGood(position, velocity, appliedVolts, current);
        inputs.shooterPosition = position.getValue();
        inputs.shooterVelocity = velocity.getValue();
        inputs.shooterAppliedVolts = appliedVolts.getValue();
        inputs.shooterCurrent = current.getValue();
    }

    @Override
    public void setVoltage(Voltage volts) {
        shooterMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        shooterMotor.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void stop() {
        shooterMotor.setControl(new NeutralOut());
    }
}
