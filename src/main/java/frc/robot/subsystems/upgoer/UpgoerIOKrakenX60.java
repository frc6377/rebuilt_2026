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
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.TunableTalonFX;

public class UpgoerIOKrakenX60 implements UpgoerIO {
    private final TunableTalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;

    public UpgoerIOKrakenX60() {
        motor = new TunableTalonFX(
                Constants.CANIDs.kUpgoerMotorCANID,
                UpgoerConstants.canBusName,
                "Upgoer/Motor",
                new Slot0Configs()
                        .withKP(UpgoerConstants.defaultKP)
                        .withKI(UpgoerConstants.defaultKI)
                        .withKD(UpgoerConstants.defaultKD)
                        .withKV(UpgoerConstants.defaultKV)
                        .withKS(UpgoerConstants.defaultKS));

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0 = motor.getTunableSlot0Configs();
        config.CurrentLimits.StatorCurrentLimit = UpgoerConstants.currentLimit.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> motor.applyConfiguration(config, 0.25));

        velocity = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        current = motor.getStatorCurrent();
        temp = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current, temp);
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    @Override
    public void updateInputs(UpgoerIOInputs inputs) {
        motor.updateTunableGains();
        BaseStatusSignal.refreshAll(velocity, appliedVolts, current, temp);

        inputs.velocity = velocity.getValue();
        inputs.appliedVoltage = appliedVolts.getValue();
        inputs.current = current.getValue();
        inputs.temp = temp.getValue();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        double rotationsPerSecond = velocity.in(RotationsPerSecond);
        motor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
