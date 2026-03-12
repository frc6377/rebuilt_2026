package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private AngularVelocity setpoint = RotationsPerSecond.zero();

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    public Command index() {
        return run(() -> {
            setpoint = IndexerConstants.kCollectorRPM;
            indexerIO.setVelocity(IndexerConstants.kCollectorRPM);
        });
    }

    public Command setCustomSpeed(double speed) {
        return runOnce(() -> indexerIO.setCustomSpeed(speed));
    }

    public Command runPercentOutput(double percent) {
        return run(() -> indexerIO.setCustomSpeed(percent));
    }

    public Command indexReverse() {
        return run(() -> {
            setpoint = IndexerConstants.kCollectorRPM.times(-1);
            indexerIO.setVelocity(IndexerConstants.kCollectorRPM.times(-1));
        });
    }

    public Command index(BooleanSupplier supplier) {
        return run(() -> {
            indexerIO.setVelocity(supplier.getAsBoolean() ? IndexerConstants.kCollectorRPM : RotationsPerSecond.zero());
        });
    }

    public Command stop() {
        return runOnce(() -> {
            setpoint = RotationsPerSecond.zero();
            indexerIO.stop();
        });
    }

    public void setRunning(boolean running) {
        if (running) {
            setpoint = IndexerConstants.kCollectorRPM;
            indexerIO.setVelocity(IndexerConstants.kCollectorRPM);
        } else {
            setpoint = RotationsPerSecond.zero();
            indexerIO.stop();
        }
    }

    @Override
    public void periodic() {
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer/Setpoint", setpoint);
        Logger.recordOutput("Indexer/Running", Math.abs(setpoint.in(RotationsPerSecond)) > 0.1);
    }
}
