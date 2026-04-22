package frc.robot.subsystems.shooter;

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
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.TunableTalonFX;
import java.util.ArrayList;
import java.util.List;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

public class BaseShooterIOKrakenX60 implements BaseShooterIO {
    private final ShooterConstants.ShooterConfig config;
    private final @NotNull TunableTalonFX flywheelMotor;
    private final @Nullable TunableTalonFX flywheelFollower;

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelCurrent;
    private final StatusSignal<Temperature> flywheelTemp;
    private final StatusSignal<AngularVelocity> followerFlywheelVelocity;
    private final StatusSignal<Voltage> followerFlywheelAppliedVolts;
    private final StatusSignal<Current> followerFlywheelCurrent;
    private final StatusSignal<Temperature> followerFlywheelTemp;

    public BaseShooterIOKrakenX60(ShooterConstants.@NotNull ShooterConfig config) {
        this.config = config;

        this.flywheelMotor = new TunableTalonFX(
                config.flywheelLeaderId(),
                config.canBusName(),
                config.name() + "/Flywheel",
                new Slot0Configs()
                        .withKP(config.flywheelKP())
                        .withKI(config.flywheelKI())
                        .withKD(config.flywheelKD())
                        .withKV(config.flywheelKV())
                        .withKS(config.flywheelKS())
                        .withKA(config.flywheelKA()));

        if (config.followerEnabled()) {
            this.flywheelFollower = new TunableTalonFX(
                    config.flywheelFollowerId(),
                    config.canBusName(),
                    config.name() + "/FlywheelFollower",
                    new Slot0Configs()
                            .withKP(config.flywheelKP())
                            .withKI(config.flywheelKI())
                            .withKD(config.flywheelKD())
                            .withKV(config.flywheelKV())
                            .withKS(config.flywheelKS())
                            .withKA(config.flywheelKA()));
        } else {
            this.flywheelFollower = null;
        }

        // Configure flywheel motor
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0 = this.flywheelLeaderConfigs();
        flywheelConfig.CurrentLimits.StatorCurrentLimit =
                config.flywheelCurrentLimitStator().in(Amps);
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = config.flywheelCurrentLimitStatorEnable();
        flywheelConfig.CurrentLimits.SupplyCurrentLimit =
                config.flywheelCurrentLimitSupply().in(Amps);
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = config.flywheelCurrentLimitSupplyEnable();
        flywheelConfig.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(
                config.flywheelClosedLoopRamp().in(Seconds));
        flywheelConfig.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(
                config.flywheelOpenLoopRamp().in(Seconds));
        flywheelConfig.withMotorOutput(
                config.outputConfigs().withInverted(config.flywheelInverted()).withNeutralMode(NeutralModeValue.Coast));

        tryUntilOk(5, () -> this.flywheelMotor.applyConfiguration(flywheelConfig, 0.25));

        if (null != flywheelFollower) {
            tryUntilOk(5, () -> this.flywheelFollower.applyConfiguration(flywheelConfig, 0.25));
            this.flywheelFollower.setControl(new Follower(this.flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }

        // Get status signals
        this.flywheelVelocity = this.flywheelMotor.getVelocity();
        this.flywheelAppliedVolts = this.flywheelMotor.getMotorVoltage();
        this.flywheelCurrent = this.flywheelMotor.getStatorCurrent();
        this.flywheelTemp = this.flywheelMotor.getDeviceTemp();
        this.followerFlywheelVelocity = this.flywheelFollower.getVelocity();
        this.followerFlywheelAppliedVolts = this.flywheelFollower.getMotorVoltage();
        this.followerFlywheelCurrent = this.flywheelFollower.getStatorCurrent();
        this.followerFlywheelTemp = this.flywheelFollower.getDeviceTemp();

        List<BaseStatusSignal> signals = new ArrayList<>();
        signals.add(this.flywheelVelocity);
        signals.add(this.flywheelAppliedVolts);
        signals.add(this.flywheelCurrent);
        signals.add(this.flywheelTemp);
        signals.add(this.followerFlywheelVelocity);
        signals.add(this.followerFlywheelAppliedVolts);
        signals.add(this.followerFlywheelCurrent);
        signals.add(this.followerFlywheelTemp);

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals.toArray(new BaseStatusSignal[0]));

        if (null != flywheelFollower) {
            ParentDevice.optimizeBusUtilizationForAll(this.flywheelMotor, this.flywheelFollower);
        } else {
            ParentDevice.optimizeBusUtilizationForAll(this.flywheelMotor);
        }
    }

    private Slot0Configs flywheelLeaderConfigs() {
        return this.flywheelMotor.getTunableSlot0Configs();
    }

    @Override
    public void updateInputs(@NotNull BaseShooterIOInputs inputs) {
        this.flywheelMotor.updateTunableGains();
        this.flywheelFollower.updateTunableGains();

        List<BaseStatusSignal> signals = new ArrayList<>();
        signals.add(this.flywheelVelocity);
        signals.add(this.flywheelAppliedVolts);
        signals.add(this.flywheelCurrent);
        signals.add(this.flywheelTemp);
        signals.add(this.followerFlywheelVelocity);
        signals.add(this.followerFlywheelAppliedVolts);
        signals.add(this.followerFlywheelCurrent);
        signals.add(this.followerFlywheelTemp);

        BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));

        inputs.flywheelVelocity = this.flywheelVelocity.getValue();
        inputs.flywheelAppliedVoltage = this.flywheelAppliedVolts.getValue();
        inputs.flywheelCurrent = this.flywheelCurrent.getValue();
        inputs.flywheelTemp = this.flywheelTemp.getValue();
        inputs.followerFlywheelVelocity = this.followerFlywheelVelocity.getValue();
        inputs.followerFlywheelAppliedVoltage = this.followerFlywheelAppliedVolts.getValue();
        inputs.followerFlywheelCurrent = this.followerFlywheelCurrent.getValue();
        inputs.followerFlywheelTemp = this.followerFlywheelTemp.getValue();

        Logger.recordOutput(
                this.config.name() + "/FlywheelVelocity (RPM)",
                this.flywheelMotor.getVelocity().getValue().in(RPM));
    }

    @Override
    public void setFlywheelVelocity(@NotNull AngularVelocity velocity) {
        this.flywheelMotor.setControl(new VelocityVoltage(velocity));

        if (null != flywheelFollower) {
            this.flywheelFollower.setControl(new Follower(this.flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    @Override
    public void setFlywheelVoltage(@NotNull Voltage voltage) {
        this.flywheelMotor.setControl(new VoltageOut(voltage));
        if (null != flywheelFollower) {
            this.flywheelFollower.setControl(new Follower(this.flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    @Override
    public void stop() {
        this.flywheelMotor.stopMotor();
        if (null != flywheelFollower) {
            this.flywheelFollower.stopMotor();
        }
    }
}
