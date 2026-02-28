package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO {

    @AutoLog
    class ExtenderIOInputs {
        public boolean isExtended = false;
        public boolean isRetracted = false;
        public Angle position = Degrees.zero();
        public Angle setpoint = Degrees.of(0.0);
        public AngularVelocity velocity = RotationsPerSecond.of(0.0);
        public Voltage motorVoltage = Volts.of(0.0);
        public Current motorCurrent = Amps.of(0.0);
        public Temperature motorTemp = Celsius.of(0.0);
        public boolean atTarget = false;
    }

    default void updateInputs(ExtenderIOInputs inputs) {}

    default void extend() {}

    default void retract() {}

    default void goToSiftAngleOne() {}

    default void goToSiftAngleTwo() {}

    default BooleanSupplier isExtended() {
        return () -> false;
    }

    default BooleanSupplier isRetracted() {
        return () -> false;
    }

    default BooleanSupplier atTarget() {
        return () -> false;
    }

    default void toggle() {}

    default void zero() {}

    default void periodic() {}
}
