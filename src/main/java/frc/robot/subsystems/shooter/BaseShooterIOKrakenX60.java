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
import frc.robot.util.TalonFXCurrentConfigurator;
import frc.robot.util.TunableTalonFX;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class BaseShooterIOKrakenX60 implements BaseShooterIO {
    private final ShooterConstants.ShooterConfig config;
    private final TunableTalonFX flywheelMotor;
    private final TunableTalonFX flywheelFollower;
    private final TalonFXCurrentConfigurator flywheelCurrentConfigurator;
    private final TalonFXCurrentConfigurator followerCurrentConfigurator;

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelCurrent;
    private final StatusSignal<Temperature> flywheelTemp;
    private final StatusSignal<AngularVelocity> followerFlywheelVelocity;
    private final StatusSignal<Voltage> followerFlywheelAppliedVolts;
    private final StatusSignal<Current> followerFlywheelCurrent;
    private final StatusSignal<Temperature> followerFlywheelTemp;

    public BaseShooterIOKrakenX60(ShooterConstants.ShooterConfig config) {
        this(config, false);
    }

    public BaseShooterIOKrakenX60(ShooterConstants.ShooterConfig config, boolean dynamicSupplyCurrentLimitEnabled) {
        this.config = config;

        flywheelMotor = new TunableTalonFX(
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
            flywheelFollower = new TunableTalonFX(
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
            flywheelFollower = null;
        }

        // Configure flywheel motor
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0 = flywheelLeaderConfigs();
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

        tryUntilOk(5, () -> flywheelMotor.applyConfiguration(flywheelConfig, 0.25));

        if (flywheelFollower != null) {
            tryUntilOk(5, () -> flywheelFollower.applyConfiguration(flywheelConfig, 0.25));
            flywheelFollower.setControl(new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }

        if (dynamicSupplyCurrentLimitEnabled) {
            flywheelCurrentConfigurator = new TalonFXCurrentConfigurator(
                    config.name() + "-Leader", flywheelMotor.getConfigurator(), flywheelConfig.CurrentLimits);
            followerCurrentConfigurator = flywheelFollower == null
                    ? null
                    : new TalonFXCurrentConfigurator(
                            config.name() + "-Follower",
                            flywheelFollower.getConfigurator(),
                            flywheelConfig.CurrentLimits);
        } else {
            flywheelCurrentConfigurator = null;
            followerCurrentConfigurator = null;
        }

        // Get status signals
        flywheelVelocity = flywheelMotor.getVelocity();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelCurrent = flywheelMotor.getSupplyCurrent();
        flywheelTemp = flywheelMotor.getDeviceTemp();
        followerFlywheelVelocity = flywheelFollower.getVelocity();
        followerFlywheelAppliedVolts = flywheelFollower.getMotorVoltage();
        followerFlywheelCurrent = flywheelFollower.getSupplyCurrent();
        followerFlywheelTemp = flywheelFollower.getDeviceTemp();

        List<BaseStatusSignal> signals = new ArrayList<>();
        signals.add(flywheelVelocity);
        signals.add(flywheelAppliedVolts);
        signals.add(flywheelCurrent);
        signals.add(flywheelTemp);
        signals.add(followerFlywheelVelocity);
        signals.add(followerFlywheelAppliedVolts);
        signals.add(followerFlywheelCurrent);
        signals.add(followerFlywheelTemp);

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals.toArray(new BaseStatusSignal[0]));

        if (flywheelFollower != null) {
            ParentDevice.optimizeBusUtilizationForAll(flywheelMotor, flywheelFollower);
        } else {
            ParentDevice.optimizeBusUtilizationForAll(flywheelMotor);
        }
    }

    private Slot0Configs flywheelLeaderConfigs() {
        return flywheelMotor.getTunableSlot0Configs();
    }

    @Override
    public void updateInputs(BaseShooterIOInputs inputs) {
        flywheelMotor.updateTunableGains();
        flywheelFollower.updateTunableGains();

        List<BaseStatusSignal> signals = new ArrayList<>();
        signals.add(flywheelVelocity);
        signals.add(flywheelAppliedVolts);
        signals.add(flywheelCurrent);
        signals.add(flywheelTemp);
        signals.add(followerFlywheelVelocity);
        signals.add(followerFlywheelAppliedVolts);
        signals.add(followerFlywheelCurrent);
        signals.add(followerFlywheelTemp);

        var refreshStatus = BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));

        inputs.flywheelVelocity = flywheelVelocity.getValue();
        inputs.flywheelAppliedVoltage = flywheelAppliedVolts.getValue();
        inputs.flywheelCurrent = flywheelCurrent.getValue();
        inputs.flywheelTemp = flywheelTemp.getValue();
        inputs.followerFlywheelVelocity = followerFlywheelVelocity.getValue();
        inputs.followerFlywheelAppliedVoltage = followerFlywheelAppliedVolts.getValue();
        inputs.followerFlywheelCurrent = followerFlywheelCurrent.getValue();
        inputs.followerFlywheelTemp = followerFlywheelTemp.getValue();
        inputs.supplyCurrentValid = refreshStatus.isOK();

        Logger.recordOutput(
                config.name() + "/FlywheelVelocity (RPM)",
                flywheelMotor.getVelocity().getValue().in(RPM));
        if (flywheelCurrentConfigurator != null) {
            logCurrentConfigurator(
                    config.name() + "/CurrentLimit/Leader-" + flywheelMotor.getDeviceID(),
                    flywheelCurrentConfigurator.snapshot());
        }
        if (followerCurrentConfigurator != null) {
            logCurrentConfigurator(
                    config.name() + "/CurrentLimit/Follower-" + flywheelFollower.getDeviceID(),
                    followerCurrentConfigurator.snapshot());
        }
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
    public void stop() {
        flywheelMotor.stopMotor();
        if (flywheelFollower != null) {
            flywheelFollower.stopMotor();
        }
    }

    @Override
    public void setSupplyCurrentLimit(double currentLimitAmps) {
        if (flywheelCurrentConfigurator != null) {
            flywheelCurrentConfigurator.requestSupplyCurrentLimit(currentLimitAmps);
        }
        if (followerCurrentConfigurator != null) {
            followerCurrentConfigurator.requestSupplyCurrentLimit(currentLimitAmps);
        }
    }

    private static void logCurrentConfigurator(String key, TalonFXCurrentConfigurator.Snapshot snapshot) {
        Logger.recordOutput(key + "/RequestedLimitAmps", snapshot.requestedLimitAmps());
        Logger.recordOutput(key + "/LastSuccessfulAcknowledgedLimitAmps", snapshot.lastSuccessfulLimitAmps());
        Logger.recordOutput(key + "/RequestedRevision", snapshot.requestedRevision());
        Logger.recordOutput(key + "/LastSuccessfulAcknowledgedRevision", snapshot.lastSuccessfulRevision());
        Logger.recordOutput(key + "/LastOutcome", snapshot.lastOutcome().name());
        Logger.recordOutput(key + "/LastStatusName", snapshot.lastStatusName());
        Logger.recordOutput(key + "/LastStatusDescription", snapshot.lastStatusDescription());
        Logger.recordOutput(key + "/LastException", snapshot.lastException());
        Logger.recordOutput(key + "/LastAttemptAgeSeconds", snapshot.lastAttemptAgeSeconds());
        Logger.recordOutput(
                key + "/LastSuccessfulAcknowledgedApplyAgeSeconds", snapshot.lastSuccessfulApplyAgeSeconds());
        Logger.recordOutput(key + "/AttemptCount", snapshot.attemptCount());
        Logger.recordOutput(key + "/SuccessCount", snapshot.successCount());
        Logger.recordOutput(key + "/FailureCount", snapshot.failureCount());
        Logger.recordOutput(key + "/ExceptionCount", snapshot.exceptionCount());
        Logger.recordOutput(key + "/RetryAttemptCount", snapshot.retryAttemptCount());
        Logger.recordOutput(key + "/DeduplicatedRequestCount", snapshot.deduplicatedRequestCount());
        Logger.recordOutput(key + "/Pending", snapshot.pending());
        Logger.recordOutput(key + "/Retrying", snapshot.retrying());
        Logger.recordOutput(key + "/InFlight", snapshot.inFlight());
        Logger.recordOutput(key + "/Closed", snapshot.closed());
        Logger.recordOutput(key + "/WorkerAlive", snapshot.workerAlive());
    }
}
