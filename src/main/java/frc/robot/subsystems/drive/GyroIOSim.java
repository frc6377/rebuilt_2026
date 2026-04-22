package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.jetbrains.annotations.NotNull;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(@NotNull GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = this.gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
                this.gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

        inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = this.gyroSimulation.getCachedGyroReadings();
    }
}
