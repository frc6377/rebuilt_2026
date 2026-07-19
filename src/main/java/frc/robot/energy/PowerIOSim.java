package frc.robot.energy;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public final class PowerIOSim implements PowerIO {
    @Override
    public void updateInputs(PowerIOInputs inputs) {
        inputs.sampleTimestampSeconds = Timer.getFPGATimestamp();
        inputs.batteryVoltage = RobotController.getBatteryVoltage();
        inputs.powerDistributionVoltage = inputs.batteryVoltage;
        inputs.totalCurrentAmps = Math.max(0.0, RobotController.getInputCurrent());
        inputs.brownedOut = RobotController.isBrownedOut();
        inputs.connected = true;
        inputs.moduleId = -1;
        inputs.moduleType = "Simulation";
    }
}
