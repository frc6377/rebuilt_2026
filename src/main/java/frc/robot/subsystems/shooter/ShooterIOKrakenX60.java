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

    // Hardware - Dual Kraken X44 for spin adjustment on each hood
    private final TalonFX leftSpinMotor;
    private final TalonFX rightSpinMotor;

    // Reference to shooter for tunable PID
    private Shooter shooter;

    // Control requests
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage spinVelocityRequest = new VelocityVoltage(0.0);

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

    // Status signals - Left spin motor
    private final StatusSignal<AngularVelocity> leftSpinVelocity;
    private final StatusSignal<Voltage> leftSpinAppliedVolts;
    private final StatusSignal<Current> leftSpinCurrent;
    private final StatusSignal<Temperature> leftSpinTemp;

    // Status signals - Right spin motor
    private final StatusSignal<AngularVelocity> rightSpinVelocity;
    private final StatusSignal<Voltage> rightSpinAppliedVolts;
    private final StatusSignal<Current> rightSpinCurrent;
    private final StatusSignal<Temperature> rightSpinTemp;

    public ShooterIOKrakenX60() {
        // Initialize flywheel motors with CAN IDs from ShooterConstants
        leftFlywheelMotor = new TalonFX(ShooterConstants.leftFlywheelMotorID, ShooterConstants.canBusName);
        rightFlywheelMotor = new TalonFX(ShooterConstants.rightFlywheelMotorID, ShooterConstants.canBusName);

        // Initialize spin motors (Kraken X44 on each hood)
        leftSpinMotor = new TalonFX(ShooterConstants.leftSpinMotorID, ShooterConstants.canBusName);
        rightSpinMotor = new TalonFX(ShooterConstants.rightSpinMotorID, ShooterConstants.canBusName);

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

        // Configure left spin motor (Kraken X44)
        var leftSpinConfig = new TalonFXConfiguration();
        leftSpinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftSpinConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftSpinConfig.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.defaultSpinKP)
                .withKI(ShooterConstants.defaultSpinKI)
                .withKD(ShooterConstants.defaultSpinKD)
                .withKV(ShooterConstants.defaultSpinKV)
                .withKS(ShooterConstants.defaultSpinKS);
        leftSpinConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.spinMotorCurrentLimit.in(Amps);
        leftSpinConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> leftSpinMotor.getConfigurator().apply(leftSpinConfig, 0.25));

        // Configure right spin motor (Kraken X44)
        var rightSpinConfig = new TalonFXConfiguration();
        rightSpinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightSpinConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Opposite of left
        rightSpinConfig.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.defaultSpinKP)
                .withKI(ShooterConstants.defaultSpinKI)
                .withKD(ShooterConstants.defaultSpinKD)
                .withKV(ShooterConstants.defaultSpinKV)
                .withKS(ShooterConstants.defaultSpinKS);
        rightSpinConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.spinMotorCurrentLimit.in(Amps);
        rightSpinConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> rightSpinMotor.getConfigurator().apply(rightSpinConfig, 0.25));

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

        // Create status signals for left spin motor
        leftSpinVelocity = leftSpinMotor.getVelocity();
        leftSpinAppliedVolts = leftSpinMotor.getMotorVoltage();
        leftSpinCurrent = leftSpinMotor.getStatorCurrent();
        leftSpinTemp = leftSpinMotor.getDeviceTemp();

        // Create status signals for right spin motor
        rightSpinVelocity = rightSpinMotor.getVelocity();
        rightSpinAppliedVolts = rightSpinMotor.getMotorVoltage();
        rightSpinCurrent = rightSpinMotor.getStatorCurrent();
        rightSpinTemp = rightSpinMotor.getDeviceTemp();

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
                rightFlywheelTemp,
                leftSpinVelocity,
                leftSpinAppliedVolts,
                leftSpinCurrent,
                leftSpinTemp,
                rightSpinVelocity,
                rightSpinAppliedVolts,
                rightSpinCurrent,
                rightSpinTemp);

        // Optimize CAN bus utilization
        ParentDevice.optimizeBusUtilizationForAll(leftFlywheelMotor, rightFlywheelMotor, leftSpinMotor, rightSpinMotor);
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
                rightFlywheelTemp,
                leftSpinVelocity,
                leftSpinAppliedVolts,
                leftSpinCurrent,
                leftSpinTemp,
                rightSpinVelocity,
                rightSpinAppliedVolts,
                rightSpinCurrent,
                rightSpinTemp);

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

        // Update left spin motor inputs
        inputs.leftSpinVelocityRPM = leftSpinVelocity.getValueAsDouble() * 60.0;
        inputs.leftSpinAppliedVolts = leftSpinAppliedVolts.getValueAsDouble();
        inputs.leftSpinCurrentAmps = leftSpinCurrent.getValueAsDouble();
        inputs.leftSpinTempCelsius = leftSpinTemp.getValueAsDouble();

        // Update right spin motor inputs
        inputs.rightSpinVelocityRPM = rightSpinVelocity.getValueAsDouble() * 60.0;
        inputs.rightSpinAppliedVolts = rightSpinAppliedVolts.getValueAsDouble();
        inputs.rightSpinCurrentAmps = rightSpinCurrent.getValueAsDouble();
        inputs.rightSpinTempCelsius = rightSpinTemp.getValueAsDouble();
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
    public void setLeftSpinVelocity(edu.wpi.first.units.measure.AngularVelocity velocity) {
        // Convert to rotations per second for TalonFX
        double rotationsPerSecond = velocity.in(RotationsPerSecond);
        leftSpinMotor.setControl(spinVelocityRequest.withVelocity(rotationsPerSecond));
    }

    @Override
    public void setRightSpinVelocity(edu.wpi.first.units.measure.AngularVelocity velocity) {
        // Convert to rotations per second for TalonFX
        double rotationsPerSecond = velocity.in(RotationsPerSecond);
        rightSpinMotor.setControl(spinVelocityRequest.withVelocity(rotationsPerSecond));
    }

    @Override
    public void stop() {
        leftFlywheelMotor.stopMotor();
        rightFlywheelMotor.stopMotor();
        leftSpinMotor.stopMotor();
        rightSpinMotor.stopMotor();
    }
}
