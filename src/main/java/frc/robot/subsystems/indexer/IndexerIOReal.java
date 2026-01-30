package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Current;

public class IndexerSubsystem extends SubsystemBase {

    private TalonFX collectorBelt; // the first belt
    private TalonFX feederBelt; // the second belt, feeds shooter

    private TalonFXConfiguration collectorBeltConfig = new TalonFXConfiguration();
    private TalonFXConfiguration feederBeltConfig = new TalonFXConfiguration();

    public IndexerSubsystem() {
        collectorBelt = new TalonFX(Constants.CANIDs.MotorIDs.kCollectorBeltID);
        feederBelt = new TalonFX(Constants.CANIDs.MotorIDs.kFeederBeltID);

        collectorBeltConfig = new TalonFXConfiguration();
        feederBeltConfig = new TalonFXConfiguration();

        collectorBeltConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feederBeltConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        collectorBelt.getConfigurator().apply(collectorBeltConfig);
        feederBelt.getConfigurator().apply(feederBeltConfig);
    }

    // ---------------------------------------------------------
    // Commands
    // ---------------------------------------------------------

    public void setCollector(double percent) {
        collectorBelt.set(percent);
    }

    public void setFeeder(double percent) {
        feederBelt.set(percent);
    }

    public void stopCollector() {
        collectorBelt.stopMotor();
    }

    public void stopFeeder() {
        feederBelt.stopMotor();
    }

    public Command runCollector(Supplier<Double> percent) {
        return Commands.runEnd(
                () -> collectorBelt.set(Math.abs(percent.get()) * IndexerConstants.kCollectorSpeed),
                () -> collectorBelt.stopMotor(),
                this);
    }

    public Command runFeeder(Supplier<Double> percent) {
        return Commands.runEnd(
                () -> feederBelt.set(Math.abs(percent.get()) * IndexerConstants.kFeederSpeed),
                () -> feederBelt.stopMotor(),
                this);
    }

    // ---------------------------------------------------------
    // Sensing Stuck Balls
    // ---------------------------------------------------------
    // TODO: Implement current sensing to detect jams

    public Current getFeederCurrent() {
        return feederBelt.getMotorCurrent().getValue();
    }

    public Current getCollectorCurrent() {
        return collectorBelt.getMotorCurrent().getValue();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Collector Output", collectorBelt.get());
        Logger.recordOutput("Feeder Output", feederBelt.get());

        Logger.recordOutput(
                "Collector Velocity (RPS)", collectorBelt.getVelocity().getValueAsDouble());
        Logger.recordOutput("Feeder Velocity (RPS)", feederBelt.getVelocity().getValueAsDouble());
    }
}
