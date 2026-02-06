package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public Distance height = Inches.zero();
    }

    default void goToHeight(Distance height) {}

    default void stop() {}

    default void set(double percent) {}

    default void updateInputs(ClimberIOInputs inputs) {}

    default void periodic() {}
}
