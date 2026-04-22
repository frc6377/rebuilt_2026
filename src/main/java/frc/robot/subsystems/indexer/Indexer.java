package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private AngularVelocity setpoint = RotationsPerSecond.zero();

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    public Command index() {
        return this.run(() -> {
            this.indexerIO.setCustomSpeed(IndexerConstants.kCollectorSpeed
                            + IndexerConstants.kCollectorVariableSpeed
                                    * Math.sin(Timer.getFPGATimestamp() * Math.PI / 8));
                })
                .finallyDo(() -> this.indexerIO.stop());
    }

    public Command setCustomSpeed(double speed) {
        return this.runOnce(() -> this.indexerIO.setCustomSpeed(speed));
    }

    public Command runPercentOutput(double percent) {
        return this.run(() -> this.indexerIO.setCustomSpeed(percent));
    }

    public Command indexReverse() {
        return this.run(() -> {
            this.setpoint = IndexerConstants.kCollectorRPM.times(-1);
            this.indexerIO.setCustomSpeed(-IndexerConstants.kCollectorSpeed);
        });
    }

    public Command index(@NotNull BooleanSupplier supplier) {
        return this.run(() -> {
            this.indexerIO.setVelocity(supplier.getAsBoolean() ? IndexerConstants.kCollectorRPM : RotationsPerSecond.zero());
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            this.setpoint = RotationsPerSecond.zero();
            this.indexerIO.stop();
        });
    }

    public void setRunning(boolean running) {
        if (running) {
            this.setpoint = IndexerConstants.kCollectorRPM;
            this.indexerIO.setVelocity(IndexerConstants.kCollectorRPM);
        } else {
            this.setpoint = RotationsPerSecond.zero();
            this.indexerIO.stop();
        }
    }

    @Override
    public void periodic() {
        this.indexerIO.updateInputs(this.inputs);
        Logger.processInputs("Indexer", this.inputs);
        Logger.recordOutput("Indexer/Setpoint", this.setpoint);
        Logger.recordOutput("Indexer/Running", 0.1 < Math.abs(setpoint.in(RotationsPerSecond)));
        Logger.recordOutput(
                "Indexer/variableSpeed",
                IndexerConstants.kCollectorSpeed
                        + (IndexerConstants.kCollectorVariableSpeed
                                * Math.sin(Timer.getFPGATimestamp() * Math.PI / 2)));
    }
}
