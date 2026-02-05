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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.TunableTalonFX;

public class ShooterIOKrakenX60 implements ShooterIO {
    // Hardware - Dual Kraken X60 for independent flywheel control
    private final TunableTalonFX leftFlywheelMotor;
    private final TunableTalonFX rightFlywheelMotor;
    private final TunableTalonFX leftFlywheelFollower;
    private final TunableTalonFX rightFlywheelFollower;

    // Hardware - Dual Kraken X44 for spin adjustment on each hood
    private final TunableTalonFX leftSpinMotor;
    private final TunableTalonFX rightSpinMotor;

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
        leftFlywheelMotor = new TunableTalonFX(
                Constants.CANIDs.kShooterFlywheelLeftMotorCANID,
                ShooterConstants.canBusName,
                "Shooter/Flywheel/Left",
                new Slot0Configs()
                        .withKP(ShooterConstants.defaultFlywheelKP)
                        .withKI(ShooterConstants.defaultFlywheelKI)
                        .withKD(ShooterConstants.defaultFlywheelKD)
                        .withKV(ShooterConstants.defaultFlywheelKV)
                        .withKS(ShooterConstants.defaultFlywheelKS));
        rightFlywheelMotor = new TunableTalonFX(
                Constants.CANIDs.kShooterFlywheelRightMotorCANID,
                ShooterConstants.canBusName,
                "Shooter/Flywheel/Right",
                new Slot0Configs()
                        .withKP(ShooterConstants.defaultFlywheelKP)
                        .withKI(ShooterConstants.defaultFlywheelKI)
                        .withKD(ShooterConstants.defaultFlywheelKD)
                        .withKV(ShooterConstants.defaultFlywheelKV)
                        .withKS(ShooterConstants.defaultFlywheelKS));
        leftFlywheelFollower = new TunableTalonFX(
                Constants.CANIDs.kShooterFlywheelLeftFollowerCANID,
                ShooterConstants.canBusName,
                "Shooter/Flywheel/LeftFollower",
                new Slot0Configs()
                        .withKP(ShooterConstants.defaultFlywheelKP)
                        .withKI(ShooterConstants.defaultFlywheelKI)
                        .withKD(ShooterConstants.defaultFlywheelKD)
                        .withKV(ShooterConstants.defaultFlywheelKV)
                        .withKS(ShooterConstants.defaultFlywheelKS));
        rightFlywheelFollower = new TunableTalonFX(
                Constants.CANIDs.kShooterFlywheelRightFollowerCANID,
                ShooterConstants.canBusName,
                "Shooter/Flywheel/RightFollower",
                new Slot0Configs()
                        .withKP(ShooterConstants.defaultFlywheelKP)
                        .withKI(ShooterConstants.defaultFlywheelKI)
                        .withKD(ShooterConstants.defaultFlywheelKD)
                        .withKV(ShooterConstants.defaultFlywheelKV)
                        .withKS(ShooterConstants.defaultFlywheelKS));

        // Initialize spin motors (Kraken X44 on each hood)
        leftSpinMotor = new TunableTalonFX(
                Constants.CANIDs.kShooterSpinMotorLeftCANID,
                ShooterConstants.canBusName,
                "Shooter/Spin/Left",
                new Slot0Configs()
                        .withKP(ShooterConstants.defaultSpinKP)
                        .withKI(ShooterConstants.defaultSpinKI)
                        .withKD(ShooterConstants.defaultSpinKD)
                        .withKV(ShooterConstants.defaultSpinKV)
                        .withKS(ShooterConstants.defaultSpinKS));
        rightSpinMotor = new TunableTalonFX(
                Constants.CANIDs.kShooterSpinMotorRightCANID,
                ShooterConstants.canBusName,
                "Shooter/Spin/Right",
                new Slot0Configs()
                        .withKP(ShooterConstants.defaultSpinKP)
                        .withKI(ShooterConstants.defaultSpinKI)
                        .withKD(ShooterConstants.defaultSpinKD)
                        .withKV(ShooterConstants.defaultSpinKV)
                        .withKS(ShooterConstants.defaultSpinKS));

        // Configure left flywheel motor
        var leftFlywheelConfig = new TalonFXConfiguration();
        leftFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftFlywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftFlywheelConfig.Slot0 = leftFlywheelMotor.getTunableSlot0Configs();
        leftFlywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.flywheelCurrentLimit.in(Amps);
        leftFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> leftFlywheelMotor.applyConfiguration(leftFlywheelConfig, 0.25));
        tryUntilOk(5, () -> leftFlywheelFollower.applyConfiguration(leftFlywheelConfig, 0.25));

        // Configure right flywheel motor (independent control, no follower)
        var rightFlywheelConfig = new TalonFXConfiguration();
        rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Opposite of left
        rightFlywheelConfig.Slot0 = rightFlywheelMotor.getTunableSlot0Configs();
        rightFlywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.flywheelCurrentLimit.in(Amps);
        rightFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> rightFlywheelMotor.applyConfiguration(rightFlywheelConfig, 0.25));
        tryUntilOk(5, () -> rightFlywheelFollower.applyConfiguration(rightFlywheelConfig, 0.25));

        // Configure left spin motor (Kraken X44)
        var leftSpinConfig = new TalonFXConfiguration();
        leftSpinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftSpinConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftSpinConfig.Slot0 = leftSpinMotor.getTunableSlot0Configs();
        leftSpinConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.spinMotorCurrentLimit.in(Amps);
        leftSpinConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> leftSpinMotor.applyConfiguration(leftSpinConfig, 0.25));

        // Configure right spin motor (Kraken X44)
        var rightSpinConfig = new TalonFXConfiguration();
        rightSpinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightSpinConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Opposite of left
        rightSpinConfig.Slot0 = rightSpinMotor.getTunableSlot0Configs();
        rightSpinConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.spinMotorCurrentLimit.in(Amps);
        rightSpinConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> rightSpinMotor.applyConfiguration(rightSpinConfig, 0.25));

        // Configure followers
        leftFlywheelFollower.setControl(new Follower(leftFlywheelMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        rightFlywheelFollower.setControl(new Follower(rightFlywheelMotor.getDeviceID(), MotorAlignmentValue.Aligned));

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
        ParentDevice.optimizeBusUtilizationForAll(
                leftFlywheelMotor,
                rightFlywheelMotor,
                leftFlywheelFollower,
                rightFlywheelFollower,
                leftSpinMotor,
                rightSpinMotor);
    }

    public void setShooter(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update tunable PID gains if shooter reference is set
        if (shooter != null) {
            leftFlywheelMotor.updateTunableGains();
            rightFlywheelMotor.updateTunableGains();
            leftSpinMotor.updateTunableGains();
            rightSpinMotor.updateTunableGains();
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
        inputs.leftFlywheelVelocity = leftFlywheelVelocity.getValue();
        inputs.leftFlywheelAppliedVoltage = leftFlywheelAppliedVolts.getValue();
        inputs.leftFlywheelCurrent = leftFlywheelCurrent.getValue();
        inputs.leftFlywheelTemp = leftFlywheelTemp.getValue();

        // Update right flywheel inputs
        inputs.rightFlywheelVelocity = rightFlywheelVelocity.getValue();
        inputs.rightFlywheelAppliedVoltage = rightFlywheelAppliedVolts.getValue();
        inputs.rightFlywheelCurrent = rightFlywheelCurrent.getValue();
        inputs.rightFlywheelTemp = rightFlywheelTemp.getValue();

        // Update left spin motor inputs
        inputs.leftSpinVelocity = leftSpinVelocity.getValue();
        inputs.leftSpinAppliedVoltage = leftSpinAppliedVolts.getValue();
        inputs.leftSpinCurrent = leftSpinCurrent.getValue();
        inputs.leftSpinTemp = leftSpinTemp.getValue();

        // Update right spin motor inputs
        inputs.rightSpinVelocity = rightSpinVelocity.getValue();
        inputs.rightSpinAppliedVoltage = rightSpinAppliedVolts.getValue();
        inputs.rightSpinCurrent = rightSpinCurrent.getValue();
        inputs.rightSpinTemp = rightSpinTemp.getValue();
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
        leftFlywheelFollower.stopMotor();
        rightFlywheelFollower.stopMotor();
        leftSpinMotor.stopMotor();
        rightSpinMotor.stopMotor();
    }
}
