package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.constants.IndexerConstants;
import frc.robot.util.TalonFXCurrentConfigurator;
import frc.robot.util.TunableTalonFX;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IndexerIOReal implements IndexerIO {
    private final TunableTalonFX indexerMotor;
    private final LoggedNetworkNumber indexerMotorOutput;
    private final LoggedNetworkNumber indexerMotorReverseOutput;
    private final TalonFXConfiguration indexerMotorConfig;
    private final Slot0Configs indexerPIDConfigs;
    private final StatusSignal<Current> supplyCurrent;
    private final TalonFXCurrentConfigurator currentConfigurator;

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
        var currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(constants.kStatorCurrentLimit())
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(constants.kSupplyCurrentLimit());
        indexerMotor.applyConfiguration(indexerMotorConfig.withCurrentLimits(currentLimits));
        currentConfigurator = new TalonFXCurrentConfigurator("Indexer", indexerMotor.getConfigurator(), currentLimits);
        supplyCurrent = indexerMotor.getSupplyCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, supplyCurrent);

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
        var supplyCurrentStatus = BaseStatusSignal.refreshAll(supplyCurrent);
        logCurrentConfigurator(
                "Indexer/CurrentLimit/Motor" + indexerMotor.getDeviceID(), currentConfigurator.snapshot());
        indexerInputs.motorOutput = Volts.of(indexerMotor.getMotorVoltage().getValueAsDouble());
        indexerInputs.motorVelocity = indexerMotor.getVelocity().getValue();
        indexerInputs.supplyCurrent = supplyCurrent.getValue();
        indexerInputs.supplyCurrentValid = supplyCurrentStatus.isOK();
    }

    @Override
    public void setSupplyCurrentLimit(double currentLimitAmps) {
        currentConfigurator.requestSupplyCurrentLimit(currentLimitAmps);
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
