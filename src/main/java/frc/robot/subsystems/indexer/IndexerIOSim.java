package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import frc.robot.subsystems.indexer.constants.IndexerConstants;

public class IndexerIOSim implements IndexerIO {
    private final FlywheelSim indexerSim;
    private final PIDController indexerController;

    private double indexerSetpointRPM = 0.0;
    private double indexerAppliedVolts = 0.0;
    private boolean closedLoopEnabled = false;

    public IndexerIOSim(IndexerConstants constants) {
        var indexerPlant = LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1), constants.rollerMOI(), constants.rollerGearing());

        indexerSim = new FlywheelSim(indexerPlant, DCMotor.getKrakenX60Foc(1));

        indexerController = new PIDController(constants.simKP(), constants.simKI(), constants.simKD());
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        if (closedLoopEnabled) {
            double indexerFB = indexerController.calculate(indexerSim.getAngularVelocityRPM(), indexerSetpointRPM);
            indexerAppliedVolts = MathUtil.clamp(indexerFB, -12.0, 12.0);
        }

        indexerSim.setInputVoltage(indexerAppliedVolts);

        indexerSim.update(Robot.defaultPeriodSecs);
        inputs.motorOutput = Volts.of(indexerAppliedVolts);
        inputs.motorVelocity = RPM.of(indexerSim.getAngularVelocityRPM());
        inputs.supplyCurrent = Amps.of(indexerSim.getCurrentDrawAmps());
        inputs.supplyCurrentValid = true;
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        closedLoopEnabled = true;
        indexerSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void setCustomSpeed(double speed) {
        closedLoopEnabled = false;
        indexerAppliedVolts = MathUtil.clamp(speed * 12.0, -12.0, 12.0);
    }

    @Override
    public void stop() {
        closedLoopEnabled = false;
        indexerSetpointRPM = 0.0;
        indexerAppliedVolts = 0.0;
        indexerSim.setInputVoltage(0.0);
    }
}
