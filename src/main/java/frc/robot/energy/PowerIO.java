package frc.robot.energy;

import org.littletonrobotics.junction.AutoLog;

public interface PowerIO {
    @AutoLog
    class PowerIOInputs {
        public double batteryVoltage = 12.0;
        public double powerDistributionVoltage = 12.0;
        public double totalCurrentAmps = 0.0;
        public boolean brownedOut = false;
        public boolean connected = false;
        public double sampleTimestampSeconds = 0.0;
        public int moduleId = -1;
        public String moduleType = "Unknown";
    }

    default void updateInputs(PowerIOInputs inputs) {}
}
