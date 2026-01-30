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
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class HoodIOKrakenX60 implements HoodIO {
    private final TalonFX motor;
    private final PositionVoltage positionRequest = new PositionVoltage(0.0);

    // Status signals
    private final StatusSignal<edu.wpi.first.units.measure.Angle> position;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;

    // Reference to hood subsystem for tunable PID
    private Hood hood;

    public HoodIOKrakenX60() {
        motor = new TalonFX(HoodConstants.motorID, HoodConstants.canBusName);

        // Configure motor
        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0 = new Slot0Configs()
                .withKP(HoodConstants.defaultKP)
                .withKI(HoodConstants.defaultKI)
                .withKD(HoodConstants.defaultKD);
        config.CurrentLimits.StatorCurrentLimit = HoodConstants.currentLimit.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = HoodConstants.gearRatio;
        tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
        tryUntilOk(5, () -> motor.setPosition(0.0, 0.25)); // Set initial position to 0

        // Create status signals
        position = motor.getPosition();
        appliedVolts = motor.getMotorVoltage();
        current = motor.getStatorCurrent();
        temp = motor.getDeviceTemp();

        // Configure update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, appliedVolts, current, temp);

        // Optimize CAN bus utilization
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    public void setHood(Hood hood) {
        this.hood = hood;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        // Update tunable PID gains if hood reference is set
        if (hood != null) {
            updatePIDGains();
        }

        // Refresh all signals
        BaseStatusSignal.refreshAll(position, appliedVolts, current, temp);

        // Update inputs (SensorToMechanismRatio already accounts for gearing)
        inputs.angleDegrees = Units.rotationsToDegrees(position.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    private void updatePIDGains() {
        var gains = new Slot0Configs()
                .withKP(hood.getKP())
                .withKI(hood.getKI())
                .withKD(hood.getKD());
        motor.getConfigurator().apply(gains);
    }

    @Override
    public void setAngle(Angle angle) {
        // Convert to rotations (SensorToMechanismRatio already accounts for gearing)
        double positionRotations = angle.in(Rotations);
        motor.setControl(positionRequest.withPosition(positionRotations));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
