package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX indexerMotor = new TalonFX(1);
    private final TalonFXConfiguration extenderMotorConfig;

    public IndexerIOReal() {
        // Real hardware-specific constructor implementation
        extenderMotorConfig = new TalonFXConfiguration();

        extenderMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        extenderMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        extenderMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = 40;
        extenderMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extenderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Configure TalonFX settings here if needed
        indexerMotor.getConfigurator().apply(extenderMotorConfig);
    }

    @Override
    public void index() {
        indexerMotor.set(IndexerConstants.kCollectorSpeed);
    }

    @Override
    public void indexReverse() {
        indexerMotor.set(-IndexerConstants.kCollectorSpeed);
    }

    @Override
    public void stop() {
        indexerMotor.set(0);
    }

    @Override
    public void setCustomSpeed(double speed) {
        indexerMotor.set(speed);
    }

    @Override
    public void updateInputs(IndexerIOInputs indexerInputs) {
        indexerInputs.updateInputs(indexerInputs);
    }
}
