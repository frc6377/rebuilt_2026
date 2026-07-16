package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.constants.IndexerConstants;
import frc.robot.util.TunableTalonFX;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IndexerIOReal implements IndexerIO {
    private final TunableTalonFX indexerMotor;
    private final LoggedNetworkNumber indexerMotorOutput;
    private final LoggedNetworkNumber indexerMotorReverseOutput;
    private final TalonFXConfiguration indexerMotorConfig;
    private final Slot0Configs indexerPIDConfigs;

    public IndexerIOReal(IndexerConstants constants) {
        // Real hardware-specific constructor implementation
        indexerPIDConfigs = new Slot0Configs();
        indexerPIDConfigs.kP = constants.kP();
        indexerPIDConfigs.kI = constants.kI();
        indexerPIDConfigs.kD = constants.kD();
        indexerMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kIndexerMotorID,
                constants.canBus(),
                "Indexer/IndexerMotor",
                indexerPIDConfigs);
        indexerMotorConfig = new TalonFXConfiguration();

        indexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = constants.voltageClosedLoopRampPeriod();
        indexerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.peakForwardTorqueCurrent();
        indexerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = constants.peakReverseTorqueCurrent();
        indexerMotorConfig.MotorOutput.Inverted = constants.motorInverted();
        indexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerMotorConfig.Slot0 = indexerPIDConfigs;
        indexerMotor.applyConfiguration(indexerMotorConfig.withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(constants.kStatorCurrentLimit())));

        indexerMotorOutput = new LoggedNetworkNumber("Indexer/IndexerMotorOutput", constants.kCollectorSpeed());
        indexerMotorReverseOutput = new LoggedNetworkNumber("Indexer/IndexerMotorReverseOutput", 0);
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
    }
}
