package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class IndexerIOSim implements IndexerIO {
    private final TalonFX indexerMotor;
    private final TalonFXSimState indexerMotorSim;
    private final TalonFXConfiguration extenderMotorConfig;

    public IndexerIOSim() {
        indexerMotor = new TalonFX(1);
        extenderMotorConfig = new TalonFXConfiguration();

        extenderMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        extenderMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        extenderMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = 40;
        extenderMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extenderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerMotor.getConfigurator().apply(extenderMotorConfig);
        indexerMotorSim = indexerMotor.getSimState();
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

    public void setCustomSpeed(double speed) {
        indexerMotor.set(speed);
    }

    @Override
    public void updateInputs(IndexerIOInputs indexerInputs) {
        indexerInputs.updateInputs(indexerInputs);
    }
}
