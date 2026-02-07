package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public Distance height = Inches.zero();

        public Angle motorPosition = Rotations.zero();

        public Voltage appliedVoltage = Volts.zero();
        public Current statorCurrent = Amps.zero();
        public Current supplyCurrent = Amps.zero();
        public double temperatureCelsius = 0.0;

        public double absoluteEncoderPosition = 0.0;

        public boolean motorConnected = true;
    }

    default void goToHeight(Distance height) {}

    default void stop() {}

    default void set(double percent) {}

    default void updateInputs(ClimberIOInputs inputs) {}

    default void periodic() {}

    default void resetToAbsolute() {}
}
