package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        // Pivot
        public Angle pivotAngle = Degrees.zero();
        public Voltage pivotAppliedVoltage = Volts.zero();
        public Current pivotStatorCurrent = Amps.zero();
        public Current pivotSupplyCurrent = Amps.zero();
        public double pivotTemperatureCelsius = 0.0;
        public boolean pivotMotorConnected = true;
        public double pivotAbsoluteEncoderPosition = 0.0;

        // Hook
        public Angle hookAngle = Degrees.zero();
        public Voltage hookAppliedVoltage = Volts.zero();
        public Current hookStatorCurrent = Amps.zero();
        public Current hookSupplyCurrent = Amps.zero();
        public double hookTemperatureCelsius = 0.0;
        public boolean hookMotorConnected = true;
        public double hookAbsoluteEncoderPosition = 0.0;
    }

    default void goToPivotAngle(Angle angle) {}

    default void setHookPercent(double percent) {}

    default void stop() {}

    default void set(double percent) {}

    default void updateInputs(ClimberIOInputs inputs) {}

    default Angle getPivotAngle() {
        return Degrees.zero();
    }

    default void periodic() {}

    default void resetToAbsolute() {}

    default void zeroEncoder() {}
}
