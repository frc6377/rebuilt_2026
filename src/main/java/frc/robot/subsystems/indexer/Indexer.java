package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.energy.FinanceDepartment;
import frc.robot.energy.MechanismStates;
import frc.robot.util.FullSubsystem;
import frc.robot.util.NerfModeController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends FullSubsystem {
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
                    setpoint = nerfModeController.getIndexerConstants().kCollectorRPM();
                    indexerIO.setCustomSpeed(
                            nerfModeController.getIndexerConstants().kCollectorSpeed()
                                    + nerfModeController.getIndexerConstants().kCollectorVariableSpeed()
                                            * Math.sin(Timer.getFPGATimestamp() * Math.PI / 8));
                })
                .finallyDo(() -> {
                    setpoint = RotationsPerSecond.zero();
                    indexerIO.stop();
                });
    }

    public Command setCustomSpeed(double speed) {
        return runOnce(() -> {
            setpoint = speed == 0.0
                    ? RotationsPerSecond.zero()
                    : nerfModeController.getIndexerConstants().kCollectorRPM();
            indexerIO.setCustomSpeed(speed);
        });
    }

    public Command runPercentOutput(double percent) {
        return run(() -> {
            setpoint = percent == 0.0
                    ? RotationsPerSecond.zero()
                    : nerfModeController.getIndexerConstants().kCollectorRPM();
            indexerIO.setCustomSpeed(percent);
        });
    }

    public Command indexReverse() {
        return run(() -> {
            setpoint = nerfModeController.getIndexerConstants().kCollectorRPM().times(-1);
            indexerIO.setCustomSpeed(-nerfModeController.getIndexerConstants().kCollectorSpeed());
        });
    }

    public Command index(BooleanSupplier supplier) {
        return run(() -> {
            setpoint = supplier.getAsBoolean()
                    ? nerfModeController.getIndexerConstants().kCollectorRPM()
                    : RotationsPerSecond.zero();
            indexerIO.setVelocity(setpoint);
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

        FinanceDepartment finance = FinanceDepartment.getInstance();
        finance.reportCurrentUsage("Indexer", true, inputs.supplyCurrent.in(Amps));
        finance.reportState(
                Math.abs(setpoint.in(RotationsPerSecond)) > 0.1
                        ? MechanismStates.Indexer.ACTIVE
                        : MechanismStates.Indexer.OFF);
    }

    @Override
    public void periodicAfterScheduler() {
        indexerIO.setSupplyCurrentLimit(FinanceDepartment.getInstance().getIndexerLimit());
    }
}
