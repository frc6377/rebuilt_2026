package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NerfModeController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final NerfModeController nerfModeController;
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private AngularVelocity setpoint = RotationsPerSecond.zero();

    public Indexer(IndexerIO indexerIO, NerfModeController nerfModeController) {
        this.indexerIO = indexerIO;
        this.nerfModeController = nerfModeController;
    }

    public Command index() {
        return run(() -> {
                    indexerIO.setCustomSpeed(
                            nerfModeController.getIndexerConstants().kCollectorSpeed()
                                    + nerfModeController.getIndexerConstants().kCollectorVariableSpeed()
                                            * Math.sin(Timer.getFPGATimestamp() * Math.PI / 8));
                })
                .finallyDo(() -> indexerIO.stop());
    }

    public Command setCustomSpeed(double speed) {
        return runOnce(() -> indexerIO.setCustomSpeed(speed));
    }

    public Command runPercentOutput(double percent) {
        return run(() -> indexerIO.setCustomSpeed(percent));
    }

    public Command indexReverse() {
        return run(() -> {
            setpoint = nerfModeController.getIndexerConstants().kCollectorRPM().times(-1);
            indexerIO.setCustomSpeed(-nerfModeController.getIndexerConstants().kCollectorSpeed());
        });
    }

    public Command index(BooleanSupplier supplier) {
        return run(() -> {
            indexerIO.setVelocity(
                    supplier.getAsBoolean()
                            ? nerfModeController.getIndexerConstants().kCollectorRPM()
                            : RotationsPerSecond.zero());
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
            setpoint = nerfModeController.getIndexerConstants().kCollectorRPM();
            indexerIO.setVelocity(nerfModeController.getIndexerConstants().kCollectorRPM());
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
        Logger.recordOutput(
                "Indexer/variableSpeed",
                nerfModeController.getIndexerConstants().kCollectorSpeed()
                        + (nerfModeController.getIndexerConstants().kCollectorVariableSpeed()
                                * Math.sin(Timer.getFPGATimestamp() * Math.PI / 2)));
    }
}
