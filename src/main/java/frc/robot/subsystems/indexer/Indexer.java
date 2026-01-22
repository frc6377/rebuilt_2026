package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor;

    public Indexer() {
        indexerMotor = new TalonFX(IndexerConstants.INDEXER_MOTOR_ID);
    }

    public void setIndexerSpeed(DoubleSupplier speed) {
        indexerMotor.set(speed.getAsDouble());
    }

    public void stopIndexer() {
        indexerMotor.stopMotor();
    }
}
