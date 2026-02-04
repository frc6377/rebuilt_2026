package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO {

    @AutoLog
    class ExtenderIOInputs {
        public boolean isExtended = false;
        public boolean isRetracted = false;
        public Angle position = Degrees.of(0.0);
        public Angle setpoint = Degrees.of(0.0);
        public Voltage motorVoltage = Volts.of(0.0);
    }

    default void updateInputs(ExtenderIOInputs inputs) {}

    default void extend() {}

    default void retract() {}

    default void setPosition(Angle position) {}

    default void zero() {}

    default void periodic() {}
}
