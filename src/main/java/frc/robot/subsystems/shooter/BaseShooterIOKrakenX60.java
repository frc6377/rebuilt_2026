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
import org.littletonrobotics.junction.Logger;

public class BaseShooterIOKrakenX60 implements BaseShooterIO {
    private final ShooterConstants.ShooterConfig config;
    private final TunableTalonFX flywheelMotor;
    private final TunableTalonFX flywheelFollower;

    private final StatusSignal<AngularVelocity> flywheelVelocity;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelCurrent;
    private final StatusSignal<Temperature> flywheelTemp;

    public BaseShooterIOKrakenX60(ShooterConstants.ShooterConfig config) {
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
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = config.flywheelInverted();
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

        tryUntilOk(5, () -> flywheelMotor.applyConfiguration(flywheelConfig, 0.25));

        if (flywheelFollower != null) {
            tryUntilOk(5, () -> flywheelFollower.applyConfiguration(flywheelConfig, 0.25));
            flywheelFollower.setControl(new Follower(flywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }

        // Get status signals
        flywheelVelocity = flywheelMotor.getVelocity();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelCurrent = flywheelMotor.getStatorCurrent();
        flywheelTemp = flywheelMotor.getDeviceTemp();

        List<BaseStatusSignal> signals = new ArrayList<>();
        signals.add(flywheelVelocity);
        signals.add(flywheelAppliedVolts);
        signals.add(flywheelCurrent);
        signals.add(flywheelTemp);

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

        List<BaseStatusSignal> signals = new ArrayList<>();
        signals.add(flywheelVelocity);
        signals.add(flywheelAppliedVolts);
        signals.add(flywheelCurrent);
        signals.add(flywheelTemp);

        BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));

        inputs.flywheelVelocity = flywheelVelocity.getValue();
        inputs.flywheelAppliedVoltage = flywheelAppliedVolts.getValue();
        inputs.flywheelCurrent = flywheelCurrent.getValue();
        inputs.flywheelTemp = flywheelTemp.getValue();

        Logger.recordOutput(
                config.name() + "/FlywheelVelocity (RPM)",
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
    public void stop() {
        flywheelMotor.stopMotor();
        if (flywheelFollower != null) {
            flywheelFollower.stopMotor();
        }
    }
}
