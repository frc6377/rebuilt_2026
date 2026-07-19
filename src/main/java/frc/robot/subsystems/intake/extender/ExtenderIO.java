package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.NeutralModeValue;
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
        public Angle setpoint = Degrees.zero();
        public AngularVelocity velocity = RotationsPerSecond.zero();
        public Voltage motorVoltage = Volts.zero();
        public Current motorCurrent = Amps.zero();
        public Current motorSupplyCurrent = Amps.zero();
        public boolean motorSupplyCurrentValid = false;
        public Temperature motorTemp = Celsius.zero();
        public boolean atTarget = false;
        public double rawEncoderDegrees = 0.0;
        public boolean atSiftCurrent = false;
    }

    default void updateInputs(ExtenderIOInputs inputs) {}

    default void extend() {}

    default void retract() {}

    default void goToSiftAngleOne() {}

    default void goToSiftAngleTwo() {}

    default void goToCustomAngleOne() {}

    default void goToCustomAngleTwo() {}

    default void toggleSift() {}

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

    default void stop() {}

    default void setPidEnabled(boolean enabled) {}

    default void setMode(NeutralModeValue mode) {}

    default void setMotorPercentage(double percent) {}

    default void periodic() {}

    default Current getCurrent() {
        return Amps.of(0.0);
    }

    default void setSupplyCurrentLimit(double currentLimitAmps) {}
}
