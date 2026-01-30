package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    class IndexerIOInputs {
        public double rollerSpeedPercentile = 0.0;
        public Voltage rollerAppliedVolts = Volts.of(0.0);
    }

    default void setCollector(double percent) {}
    default void setFeeder(double percent) {}

    default void start() {}

    default void stop() {}

    default void periodic() {}

    default void outtake() {}
}