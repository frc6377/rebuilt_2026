package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.TunableTalonFX;

public class IndexerIOReal implements IndexerIO {
    private final TunableTalonFX indexerMotor;
    private final TalonFXConfiguration indexerMotorConfig;
    private final Slot0Configs indexerPIDConfigs;

    public IndexerIOReal() {
        // Real hardware-specific constructor implementation
        indexerPIDConfigs = new Slot0Configs();
        indexerPIDConfigs.kP = IndexerConstants.PID.kP;
        indexerPIDConfigs.kI = IndexerConstants.PID.kI;
        indexerPIDConfigs.kD = IndexerConstants.PID.kD;
        indexerMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kIndexerMotorID,
                IndexerConstants.canBus,
                "Indexer/IndexerMotor",
                indexerPIDConfigs);
        indexerMotorConfig = new TalonFXConfiguration();

        indexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
                IndexerConstants.MotorConfigurationConfigs.VoltageClosedLoopRampPeriod;
        indexerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
                IndexerConstants.MotorConfigurationConfigs.PeakForwardTorqueCurrent;
        indexerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
                IndexerConstants.MotorConfigurationConfigs.PeakReverseTorqueCurrent;
        indexerMotorConfig.MotorOutput.Inverted = IndexerConstants.MotorConfigurationConfigs.MotorInverted;
        indexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerMotorConfig.Slot0 = indexerPIDConfigs;
        indexerMotor.applyConfiguration(indexerMotorConfig);
    }

    @Override
    public void stop() {
        indexerMotor.stopMotor();
    }

    @Override
    public void setCustomSpeed(double speed) {
        indexerMotor.set(speed);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        indexerMotor.setControl(new VelocityVoltage(velocity));
    }

    @Override
    public void updateInputs(IndexerIOInputs indexerInputs) {
        indexerMotor.updateTunableGains();
        indexerInputs.motorOutput = Volts.of(indexerMotor.getMotorVoltage().getValueAsDouble());
        indexerInputs.motorVelocity = RPM.of(indexerMotor.getVelocity().getValueAsDouble() * 60.0);
    }
}
