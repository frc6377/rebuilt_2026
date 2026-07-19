package frc.robot.energy;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/** Reads the installed PDP/PDH using WPILib's automatic module and hardware-type detection. */
public final class PowerIOReal implements PowerIO {
    private final PowerDistribution powerDistribution;

    public PowerIOReal() {
        this(new PowerDistribution());
    }

    PowerIOReal(PowerDistribution powerDistribution) {
        this.powerDistribution = powerDistribution;
    }

    @Override
    public void updateInputs(PowerIOInputs inputs) {
        inputs.sampleTimestampSeconds = Timer.getFPGATimestamp();
        inputs.batteryVoltage = RobotController.getBatteryVoltage();
        inputs.brownedOut = RobotController.isBrownedOut();
        inputs.moduleId = powerDistribution.getModule();
        inputs.moduleType = powerDistribution.getType().name();

        double powerDistributionVoltage = powerDistribution.getVoltage();
        double totalCurrentAmps = powerDistribution.getTotalCurrent();
        inputs.powerDistributionVoltage = powerDistributionVoltage;
        inputs.totalCurrentAmps = totalCurrentAmps;
        inputs.connected = isValidSample(totalCurrentAmps, powerDistributionVoltage);
    }

    static boolean isValidSample(double totalCurrentAmps, double powerDistributionVoltage) {
        return Double.isFinite(totalCurrentAmps)
                && totalCurrentAmps >= 0.0
                && Double.isFinite(powerDistributionVoltage)
                && powerDistributionVoltage > 1.0;
    }
}
