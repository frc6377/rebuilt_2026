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
    private boolean powerManagementActive = false;

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
                .finallyDo(this::stopIndexer);
    }

    public Command setCustomSpeed(double speed) {
        return runOnce(() -> setPercentOutput(speed));
    }

    public Command runPercentOutput(double percent) {
        return run(() -> setPercentOutput(percent)).finallyDo(this::stopIndexer);
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
        return runOnce(this::stopIndexer);
    }

    public void setRunning(boolean running) {
        if (running) {
            setpoint = nerfModeController.getIndexerConstants().kCollectorRPM();
            indexerIO.setVelocity(nerfModeController.getIndexerConstants().kCollectorRPM());
        } else {
            stopIndexer();
        }
    }

    public boolean isPowerManagementActive() {
        return powerManagementActive;
    }

    public double getSupplyCurrentAmps() {
        return inputs.supplyCurrent.in(Amps);
    }

    public boolean isSupplyCurrentValid() {
        return inputs.supplyCurrentValid;
    }

    public void setSupplyCurrentLimit(double currentLimitAmps) {
        indexerIO.setSupplyCurrentLimit(currentLimitAmps);
    }

    private void setPercentOutput(double percent) {
        setpoint = RotationsPerSecond.zero();
        powerManagementActive = Math.abs(percent) > 1e-3;
        indexerIO.setCustomSpeed(percent);
    }

    private void setVelocity(AngularVelocity velocity) {
        setpoint = velocity;
        powerManagementActive = Math.abs(velocity.in(RotationsPerSecond)) > 0.1;
        indexerIO.setVelocity(velocity);
    }

    private void stopIndexer() {
        setpoint = RotationsPerSecond.zero();
        powerManagementActive = false;
        indexerIO.stop();
    }

    @Override
    public void periodic() {
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer/Setpoint", setpoint);
        Logger.recordOutput("Indexer/Running", powerManagementActive);
        Logger.recordOutput(
                "Indexer/variableSpeed",
                nerfModeController.getIndexerConstants().kCollectorSpeed()
                        + (nerfModeController.getIndexerConstants().kCollectorVariableSpeed()
                                * Math.sin(Timer.getFPGATimestamp() * Math.PI / 2)));
    }
}
