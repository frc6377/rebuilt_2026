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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX shooterMotor;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    public ShooterIOTalonFX() {
        shooterMotor = new TalonFX(Constants.CANIDs.SHOOTER_MOTOR);

        // Apply configuration
        TalonFXConfiguration config = Constants.ShooterConstants.shooterTalonFXConfiguration;

        // Ensure neutral mode is brake or coast as desired (override if needed, but using constant for now)
        // config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterMotor.getConfigurator().apply(config);

        // Signals
        position = shooterMotor.getPosition();
        velocity = shooterMotor.getVelocity();
        appliedVolts = shooterMotor.getMotorVoltage();
        current = shooterMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
        shooterMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
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
        shooterMotor.setControl(voltageRequest.withOutput(0.0));
    }
}
