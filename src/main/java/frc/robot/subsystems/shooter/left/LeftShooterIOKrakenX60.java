package frc.robot.subsystems.shooter.left;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.TunableTalonFX;
import org.littletonrobotics.junction.Logger;

public class LeftShooterIOKrakenX60 implements LeftShooterIO {
    private final TunableTalonFX flywheelMotor;
    private final TunableTalonFX flywheelFollower;
    private final TunableTalonFX spinMotor;

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelCurrent;
    private final StatusSignal<Temperature> flywheelTemp;

    private final StatusSignal<AngularVelocity> spinVelocity;
    private final StatusSignal<Voltage> spinAppliedVolts;
    private final StatusSignal<Current> spinCurrent;
    private final StatusSignal<Temperature> spinTemp;

    public LeftShooterIOKrakenX60() {
        InvertedValue flywheelInverted = LeftShooterConstants.flywheelInverted;
        InvertedValue spinInverted = LeftShooterConstants.spinInverted;

        flywheelMotor = new TunableTalonFX(
                LeftShooterConstants.flywheelLeaderId,
                LeftShooterConstants.canBusName,
                "LeftShooter/Flywheel",
                new Slot0Configs()
                        .withKP(LeftShooterConstants.flywheelKP)
                        .withKI(LeftShooterConstants.flywheelKI)
                        .withKD(LeftShooterConstants.flywheelKD)
                        .withKV(LeftShooterConstants.flywheelKV)
                        .withKS(LeftShooterConstants.flywheelKS)
                        .withKA(LeftShooterConstants.flywheelKA));

        if (LeftShooterConstants.followerEnabled) {
            flywheelFollower = new TunableTalonFX(
                    LeftShooterConstants.flywheelFollowerId,
                    LeftShooterConstants.canBusName,
                    "LeftShooter/FlywheelFollower",
                    new Slot0Configs()
                            .withKP(LeftShooterConstants.flywheelKP)
                            .withKI(LeftShooterConstants.flywheelKI)
                            .withKD(LeftShooterConstants.flywheelKD)
                            .withKV(LeftShooterConstants.flywheelKV)
                            .withKS(LeftShooterConstants.flywheelKS)
                            .withKA(LeftShooterConstants.flywheelKA));
        } else {
            flywheelFollower = null;
        }

        if (LeftShooterConstants.spinMotorEnabled) {
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
        } else {
            spinMotor = null;
        }

        // Configure flywheel motor
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = flywheelInverted;
        flywheelConfig.Slot0 = flywheelMotor.getTunableSlot0Configs();
        flywheelConfig.CurrentLimits.StatorCurrentLimit = LeftShooterConstants.flywheelCurrentLimitStator.in(Amps);
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = LeftShooterConstants.flywheelCurrentLimitStatorEnable;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = LeftShooterConstants.flywheelCurrentLimitSupply.in(Amps);
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = LeftShooterConstants.flywheelCurrentLimitSupplyEnable;
        flywheelConfig.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(
                LeftShooterConstants.flywheelClosedLoopRamp.in(Seconds));
        flywheelConfig.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(
                LeftShooterConstants.flywheelOpenLoopRamp.in(Seconds));

        tryUntilOk(5, () -> flywheelMotor.applyConfiguration(flywheelConfig, 0.25));

        if (flywheelFollower != null) {
            tryUntilOk(5, () -> flywheelFollower.applyConfiguration(flywheelConfig, 0.25));
            flywheelFollower.setControl(new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }

        if (spinMotor != null) {
            var spinConfig = new TalonFXConfiguration();
            spinConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            spinConfig.MotorOutput.Inverted = spinInverted;
            spinConfig.Slot0 = spinMotor.getTunableSlot0Configs();
            spinConfig.CurrentLimits.StatorCurrentLimit = LeftShooterConstants.spinCurrentLimitStator.in(Amps);
            spinConfig.CurrentLimits.StatorCurrentLimitEnable = LeftShooterConstants.spinCurrentLimitStatorEnable;
            spinConfig.CurrentLimits.SupplyCurrentLimit = LeftShooterConstants.spinCurrentLimitSupply.in(Amps);
            spinConfig.CurrentLimits.SupplyCurrentLimitEnable = LeftShooterConstants.spinCurrentLimitSupplyEnable;
            spinConfig.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(
                    LeftShooterConstants.spinClosedLoopRamp.in(Seconds));
            spinConfig.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(LeftShooterConstants.spinOpenLoopRamp.in(Seconds));
            tryUntilOk(5, () -> spinMotor.applyConfiguration(spinConfig, 0.25));
        }
        // Get status signals
        flywheelVelocity = flywheelMotor.getVelocity();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelCurrent = flywheelMotor.getStatorCurrent();
        flywheelTemp = flywheelMotor.getDeviceTemp();

        if (spinMotor != null) {
            spinVelocity = spinMotor.getVelocity();
            spinAppliedVolts = spinMotor.getMotorVoltage();
            spinCurrent = spinMotor.getStatorCurrent();
            spinTemp = spinMotor.getDeviceTemp();
        } else {
            spinVelocity = null;
            spinAppliedVolts = null;
            spinCurrent = null;
            spinTemp = null;
        }

        var signals = new java.util.ArrayList<BaseStatusSignal>();
        signals.add(flywheelVelocity);
        signals.add(flywheelAppliedVolts);
        signals.add(flywheelCurrent);
        signals.add(flywheelTemp);
        if (spinMotor != null) {
            signals.add(spinVelocity);
            signals.add(spinAppliedVolts);
            signals.add(spinCurrent);
            signals.add(spinTemp);
        }

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals.toArray(new BaseStatusSignal[0]));

        if (flywheelFollower != null && spinMotor != null) {
            ParentDevice.optimizeBusUtilizationForAll(flywheelMotor, flywheelFollower, spinMotor);
        } else if (flywheelFollower != null) {
            ParentDevice.optimizeBusUtilizationForAll(flywheelMotor, flywheelFollower);
        } else if (spinMotor != null) {
            ParentDevice.optimizeBusUtilizationForAll(flywheelMotor, spinMotor);
        } else {
            ParentDevice.optimizeBusUtilizationForAll(flywheelMotor);
        }
    }

    @Override
    public void updateInputs(LeftShooterIOInputs inputs) {
        flywheelMotor.updateTunableGains();
        if (spinMotor != null) {
            spinMotor.updateTunableGains();
        }

        var signals = new java.util.ArrayList<BaseStatusSignal>();
        signals.add(flywheelVelocity);
        signals.add(flywheelAppliedVolts);
        signals.add(flywheelCurrent);
        signals.add(flywheelTemp);
        if (spinMotor != null) {
            signals.add(spinVelocity);
            signals.add(spinAppliedVolts);
            signals.add(spinCurrent);
            signals.add(spinTemp);
        }

        BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));

        inputs.flywheelVelocity = flywheelVelocity.getValue();
        inputs.flywheelAppliedVoltage = flywheelAppliedVolts.getValue();
        inputs.flywheelCurrent = flywheelCurrent.getValue();
        inputs.flywheelTemp = flywheelTemp.getValue();

        if (spinMotor != null) {
            inputs.spinVelocity = spinVelocity.getValue();
            inputs.spinAppliedVoltage = spinAppliedVolts.getValue();
            inputs.spinCurrent = spinCurrent.getValue();
            inputs.spinTemp = spinTemp.getValue();
        } else {
            inputs.spinVelocity = RPM.of(0.0);
            inputs.spinAppliedVoltage = Volts.of(0.0);
            inputs.spinCurrent = Amps.of(0.0);
            inputs.spinTemp = Celsius.of(0.0);
        }
        Logger.recordOutput(
                "LeftShooter/FlywheelVelocity (RPM)",
                flywheelMotor.getVelocity().getValue().in(RPM));
    }

    @Override
    public void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelMotor.setControl(new VelocityVoltage(velocity));

        if (flywheelFollower != null) {
            flywheelFollower.setControl(new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    @Override
    public void setFlywheelVoltage(Voltage voltage) {
        flywheelMotor.setControl(new VoltageOut(voltage));
        if (flywheelFollower != null) {
            flywheelFollower.setControl(new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    @Override
    public void setSpinVelocity(AngularVelocity velocity) {
        if (spinMotor != null) {
            spinMotor.setControl(new VelocityVoltage(velocity));
        }
    }

    @Override
    public void stop() {
        flywheelMotor.stopMotor();
        if (flywheelFollower != null) {
            flywheelFollower.stopMotor();
        }
        if (spinMotor != null) {
            spinMotor.stopMotor();
        }
    }
}
