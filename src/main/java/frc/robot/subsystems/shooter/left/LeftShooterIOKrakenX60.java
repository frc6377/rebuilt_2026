package frc.robot.subsystems.shooter.left;

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
import frc.robot.util.TunableTalonFX;

public class LeftShooterIOKrakenX60 implements LeftShooterIO {
    private final TunableTalonFX flywheelMotor;
    private final TunableTalonFX flywheelFollower;
    private final TunableTalonFX spinMotor;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage spinVelocityRequest = new VelocityVoltage(0.0);

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelCurrent;
    private final StatusSignal<Temperature> flywheelTemp;

    private final StatusSignal<AngularVelocity> spinVelocity;
    private final StatusSignal<Voltage> spinAppliedVolts;
    private final StatusSignal<Current> spinCurrent;
    private final StatusSignal<Temperature> spinTemp;

    public LeftShooterIOKrakenX60() {
        InvertedValue flywheelInverted = LeftShooterConstants.flywheelInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        InvertedValue spinInverted = LeftShooterConstants.spinInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        flywheelMotor = new TunableTalonFX(
                LeftShooterConstants.flywheelLeaderId,
                LeftShooterConstants.canBusName,
                "LeftShooter/Flywheel",
                new Slot0Configs()
                        .withKP(LeftShooterConstants.flywheelKP)
                        .withKI(LeftShooterConstants.flywheelKI)
                        .withKD(LeftShooterConstants.flywheelKD)
                        .withKV(LeftShooterConstants.flywheelKV)
                        .withKS(LeftShooterConstants.flywheelKS));

        flywheelFollower = new TunableTalonFX(
                LeftShooterConstants.flywheelFollowerId,
                LeftShooterConstants.canBusName,
                "LeftShooter/FlywheelFollower",
                new Slot0Configs()
                        .withKP(LeftShooterConstants.flywheelKP)
                        .withKI(LeftShooterConstants.flywheelKI)
                        .withKD(LeftShooterConstants.flywheelKD)
                        .withKV(LeftShooterConstants.flywheelKV)
                        .withKS(LeftShooterConstants.flywheelKS));

        spinMotor = new TunableTalonFX(
                LeftShooterConstants.spinMotorId,
                LeftShooterConstants.canBusName,
                "LeftShooter/Spin",
                new Slot0Configs()
                        .withKP(LeftShooterConstants.spinKP)
                        .withKI(LeftShooterConstants.spinKI)
                        .withKD(LeftShooterConstants.spinKD)
                        .withKV(LeftShooterConstants.spinKV)
                        .withKS(LeftShooterConstants.spinKS));

        // Configure flywheel motor
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = flywheelInverted;
        flywheelConfig.Slot0 = flywheelMotor.getTunableSlot0Configs();
        flywheelConfig.CurrentLimits.StatorCurrentLimit = LeftShooterConstants.flywheelCurrentLimitStator.in(Amps);
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = LeftShooterConstants.flywheelCurrentLimitStatorEnable;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = LeftShooterConstants.flywheelCurrentLimitSupply.in(Amps);
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = LeftShooterConstants.flywheelCurrentLimitSupplyEnable;
        tryUntilOk(5, () -> flywheelMotor.applyConfiguration(flywheelConfig, 0.25));
        tryUntilOk(5, () -> flywheelFollower.applyConfiguration(flywheelConfig, 0.25));

        // Configure spin motor
        var spinConfig = new TalonFXConfiguration();
        spinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        spinConfig.MotorOutput.Inverted = spinInverted;
        spinConfig.Slot0 = spinMotor.getTunableSlot0Configs();
        spinConfig.CurrentLimits.StatorCurrentLimit = LeftShooterConstants.spinCurrentLimitStator.in(Amps);
        spinConfig.CurrentLimits.StatorCurrentLimitEnable = LeftShooterConstants.spinCurrentLimitStatorEnable;
        spinConfig.CurrentLimits.SupplyCurrentLimit = LeftShooterConstants.spinCurrentLimitSupply.in(Amps);
        spinConfig.CurrentLimits.SupplyCurrentLimitEnable = LeftShooterConstants.spinCurrentLimitSupplyEnable;
        tryUntilOk(5, () -> spinMotor.applyConfiguration(spinConfig, 0.25));

        // Set follower
        flywheelFollower.setControl(new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        // Get status signals
        flywheelVelocity = flywheelMotor.getVelocity();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelCurrent = flywheelMotor.getStatorCurrent();
        flywheelTemp = flywheelMotor.getDeviceTemp();

        spinVelocity = spinMotor.getVelocity();
        spinAppliedVolts = spinMotor.getMotorVoltage();
        spinCurrent = spinMotor.getStatorCurrent();
        spinTemp = spinMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                flywheelVelocity,
                flywheelAppliedVolts,
                flywheelCurrent,
                flywheelTemp,
                spinVelocity,
                spinAppliedVolts,
                spinCurrent,
                spinTemp);

        ParentDevice.optimizeBusUtilizationForAll(flywheelMotor, flywheelFollower, spinMotor);
    }

    @Override
    public void updateInputs(LeftShooterIOInputs inputs) {
        flywheelMotor.updateTunableGains();
        spinMotor.updateTunableGains();

        BaseStatusSignal.refreshAll(
                flywheelVelocity,
                flywheelAppliedVolts,
                flywheelCurrent,
                flywheelTemp,
                spinVelocity,
                spinAppliedVolts,
                spinCurrent,
                spinTemp);

        inputs.flywheelVelocity = flywheelVelocity.getValue();
        inputs.flywheelAppliedVoltage = flywheelAppliedVolts.getValue();
        inputs.flywheelCurrent = flywheelCurrent.getValue();
        inputs.flywheelTemp = flywheelTemp.getValue();
        inputs.spinVelocity = spinVelocity.getValue();
        inputs.spinAppliedVoltage = spinAppliedVolts.getValue();
        inputs.spinCurrent = spinCurrent.getValue();
        inputs.spinTemp = spinTemp.getValue();
    }

    @Override
    public void setFlywheelVelocity(AngularVelocity velocity) {
        double rotationsPerSecond = velocity.in(RotationsPerSecond);
        flywheelMotor.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    @Override
    public void setSpinVelocity(AngularVelocity velocity) {
        double rotationsPerSecond = velocity.in(RotationsPerSecond);
        spinMotor.setControl(spinVelocityRequest.withVelocity(rotationsPerSecond));
    }

    @Override
    public void stop() {
        flywheelMotor.stopMotor();
        flywheelFollower.stopMotor();
        spinMotor.stopMotor();
    }
}
