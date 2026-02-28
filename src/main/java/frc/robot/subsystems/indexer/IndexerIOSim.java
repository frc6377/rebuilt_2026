package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class IndexerIOSim implements IndexerIO {
    private final FlywheelSim indexerSim;
    private final PIDController indexerController;

    private double indexerSetpointRPM = 0.0;
    private double indexerAppliedVolts = 0.0;

    public IndexerIOSim() {
        var indexerPlant =
                LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(2), 0.5 * 0.5 * 0.01 * 0.01, 1.0);

        indexerSim = new FlywheelSim(indexerPlant, DCMotor.getKrakenX60Foc(1));

        indexerController = new PIDController(0.001, 0.0005, 0.0);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        double indexerFB = indexerController.calculate(indexerSim.getAngularVelocityRPM(), indexerSetpointRPM);
        indexerAppliedVolts = MathUtil.clamp(indexerFB, -12.0, 12.0);

        indexerSim.setInputVoltage(indexerAppliedVolts);

        indexerSim.update(Robot.defaultPeriodSecs);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        indexerSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void stop() {
        indexerSetpointRPM = 0.0;
        indexerAppliedVolts = 0.0;
        indexerSim.setInputVoltage(0.0);
    }
}
