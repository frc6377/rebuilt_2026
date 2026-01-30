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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOKrakenX60 implements ShooterIO {
    // Hardware - Dual Kraken X60 for independent flywheel control
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;

    // Reference to shooter for tunable PID
    private Shooter shooter;

    // Control requests
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    // Status signals - Left flywheel
    private final StatusSignal<AngularVelocity> leftFlywheelVelocity;
    private final StatusSignal<Voltage> leftFlywheelAppliedVolts;
    private final StatusSignal<Current> leftFlywheelCurrent;
    private final StatusSignal<Temperature> leftFlywheelTemp;

    // Status signals - Right flywheel
    private final StatusSignal<AngularVelocity> rightFlywheelVelocity;
    private final StatusSignal<Voltage> rightFlywheelAppliedVolts;
    private final StatusSignal<Current> rightFlywheelCurrent;
    private final StatusSignal<Temperature> rightFlywheelTemp;

    public ShooterIOKrakenX60() {
        // Initialize motors with CAN IDs from ShooterConstants
        leftFlywheelMotor = new TalonFX(ShooterConstants.leftFlywheelMotorID, ShooterConstants.canBusName);
        rightFlywheelMotor = new TalonFX(ShooterConstants.rightFlywheelMotorID, ShooterConstants.canBusName);

        // Configure left flywheel motor
        var leftFlywheelConfig = new TalonFXConfiguration();
        leftFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftFlywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftFlywheelConfig.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.defaultFlywheelKP)
                .withKI(ShooterConstants.defaultFlywheelKI)
                .withKD(ShooterConstants.defaultFlywheelKD)
                .withKV(ShooterConstants.defaultFlywheelKV)
                .withKS(ShooterConstants.defaultFlywheelKS);
        leftFlywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.flywheelCurrentLimit.in(Amps);
        leftFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> leftFlywheelMotor.getConfigurator().apply(leftFlywheelConfig, 0.25));

        // Configure right flywheel motor (independent control, no follower)
        var rightFlywheelConfig = new TalonFXConfiguration();
        rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Opposite of left
        rightFlywheelConfig.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.defaultFlywheelKP)
                .withKI(ShooterConstants.defaultFlywheelKI)
                .withKD(ShooterConstants.defaultFlywheelKD)
                .withKV(ShooterConstants.defaultFlywheelKV)
                .withKS(ShooterConstants.defaultFlywheelKS);
        rightFlywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.flywheelCurrentLimit.in(Amps);
        rightFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> rightFlywheelMotor.getConfigurator().apply(rightFlywheelConfig, 0.25));

        // Create status signals for left flywheel
        leftFlywheelVelocity = leftFlywheelMotor.getVelocity();
        leftFlywheelAppliedVolts = leftFlywheelMotor.getMotorVoltage();
        leftFlywheelCurrent = leftFlywheelMotor.getStatorCurrent();
        leftFlywheelTemp = leftFlywheelMotor.getDeviceTemp();

        // Create status signals for right flywheel
        rightFlywheelVelocity = rightFlywheelMotor.getVelocity();
        rightFlywheelAppliedVolts = rightFlywheelMotor.getMotorVoltage();
        rightFlywheelCurrent = rightFlywheelMotor.getStatorCurrent();
        rightFlywheelTemp = rightFlywheelMotor.getDeviceTemp();

        // Configure update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                leftFlywheelVelocity,
                leftFlywheelAppliedVolts,
                leftFlywheelCurrent,
                leftFlywheelTemp,
                rightFlywheelVelocity,
                rightFlywheelAppliedVolts,
                rightFlywheelCurrent,
                rightFlywheelTemp);

        // Optimize CAN bus utilization
        ParentDevice.optimizeBusUtilizationForAll(leftFlywheelMotor, rightFlywheelMotor);
    }

    public void setShooter(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update tunable PID gains if shooter reference is set
        if (shooter != null) {
            updatePIDGains();
        }

        // Refresh all signals
        BaseStatusSignal.refreshAll(
                leftFlywheelVelocity,
                leftFlywheelAppliedVolts,
                leftFlywheelCurrent,
                leftFlywheelTemp,
                rightFlywheelVelocity,
                rightFlywheelAppliedVolts,
                rightFlywheelCurrent,
                rightFlywheelTemp);

        // Update left flywheel inputs
        inputs.leftFlywheelVelocityRPM = leftFlywheelVelocity.getValueAsDouble() * 60.0;
        inputs.leftFlywheelAppliedVolts = leftFlywheelAppliedVolts.getValueAsDouble();
        inputs.leftFlywheelCurrentAmps = leftFlywheelCurrent.getValueAsDouble();
        inputs.leftFlywheelTempCelsius = leftFlywheelTemp.getValueAsDouble();

        // Update right flywheel inputs
        inputs.rightFlywheelVelocityRPM = rightFlywheelVelocity.getValueAsDouble() * 60.0;
        inputs.rightFlywheelAppliedVolts = rightFlywheelAppliedVolts.getValueAsDouble();
        inputs.rightFlywheelCurrentAmps = rightFlywheelCurrent.getValueAsDouble();
        inputs.rightFlywheelTempCelsius = rightFlywheelTemp.getValueAsDouble();
    }

    private void updatePIDGains() {
        // Update flywheel PID gains
        var flywheelGains = new Slot0Configs()
                .withKP(shooter.getFlywheelKP())
                .withKI(shooter.getFlywheelKI())
                .withKD(shooter.getFlywheelKD())
                .withKV(shooter.getFlywheelKV())
                .withKS(shooter.getFlywheelKS());

        leftFlywheelMotor.getConfigurator().apply(flywheelGains);
        rightFlywheelMotor.getConfigurator().apply(flywheelGains);
    }

    @Override
    public void setLeftFlywheelVelocity(edu.wpi.first.units.measure.AngularVelocity velocity) {
        // Convert to rotations per second for TalonFX
        double rotationsPerSecond = velocity.in(RotationsPerSecond);
        leftFlywheelMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    @Override
    public void setRightFlywheelVelocity(edu.wpi.first.units.measure.AngularVelocity velocity) {
        // Convert to rotations per second for TalonFX
        double rotationsPerSecond = velocity.in(RotationsPerSecond);
        rightFlywheelMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    @Override
    public void stop() {
        leftFlywheelMotor.stopMotor();
        rightFlywheelMotor.stopMotor();
    }
}
