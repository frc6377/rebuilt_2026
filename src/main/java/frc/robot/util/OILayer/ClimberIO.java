package frc.robot.util.OILayer;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {}

    default void goToHeight(Angle height) {}

    default void updateInputs(ClimberIOInputs inputs) {}

    default void stop() {}

    default void set(double percent) {}


}