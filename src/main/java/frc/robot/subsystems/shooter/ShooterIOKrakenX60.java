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

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOKrakenX60 implements ShooterIO {
    // Hardware - Dual Kraken X60 for independent flywheel control
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;
    private final TalonFX hoodMotor;

    // Control requests
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final com.ctre.phoenix6.controls.PositionVoltage positionRequest =
            new com.ctre.phoenix6.controls.PositionVoltage(0.0);

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

    // Status signals - Hood
    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<Voltage> hoodAppliedVolts;
    private final StatusSignal<Current> hoodCurrent;
    private final StatusSignal<Temperature> hoodTemp;

    public ShooterIOKrakenX60() {
        // Initialize motors with CAN IDs from ShooterConstants
        leftFlywheelMotor = new TalonFX(ShooterConstants.leftFlywheelMotorID, ShooterConstants.canBusName);
        rightFlywheelMotor = new TalonFX(ShooterConstants.rightFlywheelMotorID, ShooterConstants.canBusName);
        hoodMotor = new TalonFX(ShooterConstants.hoodMotorID, ShooterConstants.canBusName);

        // Configure left flywheel motor
        var leftFlywheelConfig = new TalonFXConfiguration();
        leftFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftFlywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftFlywheelConfig.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.flywheelKP)
                .withKI(ShooterConstants.flywheelKI)
                .withKD(ShooterConstants.flywheelKD)
                .withKV(ShooterConstants.flywheelKV)
                .withKS(ShooterConstants.flywheelKS);
        leftFlywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.flywheelCurrentLimit;
        leftFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> leftFlywheelMotor.getConfigurator().apply(leftFlywheelConfig, 0.25));

        // Configure right flywheel motor (independent control, no follower)
        var rightFlywheelConfig = new TalonFXConfiguration();
        rightFlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightFlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Opposite of left
        rightFlywheelConfig.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.flywheelKP)
                .withKI(ShooterConstants.flywheelKI)
                .withKD(ShooterConstants.flywheelKD)
                .withKV(ShooterConstants.flywheelKV)
                .withKS(ShooterConstants.flywheelKS);
        rightFlywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.flywheelCurrentLimit;
        rightFlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> rightFlywheelMotor.getConfigurator().apply(rightFlywheelConfig, 0.25));

        // Configure hood motor (Kraken x44 for position control)
        var hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodConfig.Slot0 = new Slot0Configs()
                .withKP(ShooterConstants.hoodKP)
                .withKI(ShooterConstants.hoodKI)
                .withKD(ShooterConstants.hoodKD);
        hoodConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.hoodCurrentLimit;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.Feedback.SensorToMechanismRatio = ShooterConstants.hoodGearRatio;
        tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
        tryUntilOk(5, () -> hoodMotor.setPosition(0.0, 0.25)); // Set initial position to 0

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

        // Create status signals for hood
        hoodPosition = hoodMotor.getPosition();
        hoodAppliedVolts = hoodMotor.getMotorVoltage();
        hoodCurrent = hoodMotor.getStatorCurrent();
        hoodTemp = hoodMotor.getDeviceTemp();

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
                hoodPosition,
                hoodAppliedVolts,
                hoodCurrent,
                hoodTemp);

        // Optimize CAN bus utilization
        ParentDevice.optimizeBusUtilizationForAll(leftFlywheelMotor, rightFlywheelMotor, hoodMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
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
                hoodPosition,
                hoodAppliedVolts,
                hoodCurrent,
                hoodTemp);

        // Update left flywheel inputs
        inputs.leftFlywheelVelocityRPM =
                Units.radiansToRotations(leftFlywheelVelocity.getValueAsDouble()) * 60.0;
        inputs.leftFlywheelAppliedVolts = leftFlywheelAppliedVolts.getValueAsDouble();
        inputs.leftFlywheelCurrentAmps = leftFlywheelCurrent.getValueAsDouble();
        inputs.leftFlywheelTempCelsius = leftFlywheelTemp.getValueAsDouble();

        // Update right flywheel inputs
        inputs.rightFlywheelVelocityRPM =
                Units.radiansToRotations(rightFlywheelVelocity.getValueAsDouble()) * 60.0;
        inputs.rightFlywheelAppliedVolts = rightFlywheelAppliedVolts.getValueAsDouble();
        inputs.rightFlywheelCurrentAmps = rightFlywheelCurrent.getValueAsDouble();
        inputs.rightFlywheelTempCelsius = rightFlywheelTemp.getValueAsDouble();

        // Update hood inputs
        inputs.hoodAngleDegrees =
                Units.rotationsToDegrees(hoodPosition.getValueAsDouble() / ShooterConstants.hoodGearRatio);
        inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
        inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();
        inputs.hoodTempCelsius = hoodTemp.getValueAsDouble();
    }

    @Override
    public void setLeftFlywheelVelocity(double velocityRPM) {
        // Convert RPM to rotations per second for TalonFX
        double rotationsPerSecond = velocityRPM / 60.0;
        leftFlywheelMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    @Override
    public void setRightFlywheelVelocity(double velocityRPM) {
        // Convert RPM to rotations per second for TalonFX
        double rotationsPerSecond = velocityRPM / 60.0;
        rightFlywheelMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    @Override
    public void setHoodAngle(double angleDegrees) {
        // Convert degrees to rotations with gear ratio
        double positionRotations = Units.degreesToRotations(angleDegrees) * ShooterConstants.hoodGearRatio;
        hoodMotor.setControl(positionRequest.withPosition(positionRotations));
    }

    @Override
    public void stop() {
        leftFlywheelMotor.stopMotor();
        rightFlywheelMotor.stopMotor();
        hoodMotor.stopMotor();
    }
}
