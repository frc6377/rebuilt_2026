package frc.robot.util.OILayer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
    class IntakeIOInputs {
        public double rollerSpeedPercentile = 0.0;
        public Voltage rollerAppliedVolts = Volts.of(0.0);
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setRollerSpeed(double speed) {}
    
}
