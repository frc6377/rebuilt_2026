package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.TunableTalonFX;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IndexerIOReal implements IndexerIO {
    private final @NotNull TunableTalonFX indexerMotor;

    public IndexerIOReal() {
        // Real hardware-specific constructor implementation
        @NotNull Slot0Configs indexerPIDConfigs = new Slot0Configs();
        indexerPIDConfigs.kP = IndexerConstants.PID.kP;
        indexerPIDConfigs.kI = IndexerConstants.PID.kI;
        indexerPIDConfigs.kD = IndexerConstants.PID.kD;
        this.indexerMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kIndexerMotorID,
                IndexerConstants.canBus,
                "Indexer/IndexerMotor",
                indexerPIDConfigs);
        @NotNull TalonFXConfiguration indexerMotorConfig = new TalonFXConfiguration();

        indexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
                IndexerConstants.MotorConfigurationConfigs.VoltageClosedLoopRampPeriod;
        indexerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
                IndexerConstants.MotorConfigurationConfigs.PeakForwardTorqueCurrent;
        indexerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
                IndexerConstants.MotorConfigurationConfigs.PeakReverseTorqueCurrent;
        indexerMotorConfig.MotorOutput.Inverted = IndexerConstants.MotorConfigurationConfigs.MotorInverted;
        indexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerMotorConfig.Slot0 = indexerPIDConfigs;
        this.indexerMotor.applyConfiguration(indexerMotorConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(IndexerConstants.MotorConfigurationConfigs.kStatorCurrentLimit)));

        @NotNull LoggedNetworkNumber indexerMotorOutput = new LoggedNetworkNumber("Indexer/IndexerMotorOutput", IndexerConstants.kCollectorSpeed);
        @NotNull LoggedNetworkNumber indexerMotorReverseOutput = new LoggedNetworkNumber("Indexer/IndexerMotorReverseOutput", 0);
    }

    @Override
    public void stop() {
        this.indexerMotor.stopMotor();
    }

    @Override
    public void setCustomSpeed(double speed) {
        this.indexerMotor.set(speed);
    }

    @Override
    public void setVelocity(@NotNull AngularVelocity velocity) {
        this.indexerMotor.setControl(new VelocityVoltage(velocity));
    }

    @Override
    public void updateInputs(@NotNull IndexerIOInputs indexerInputs) {
        this.indexerMotor.updateTunableGains();
        indexerInputs.motorOutput = Volts.of(this.indexerMotor.getMotorVoltage().getValueAsDouble());
    }
}
