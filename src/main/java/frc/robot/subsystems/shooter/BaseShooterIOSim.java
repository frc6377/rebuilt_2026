package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class BaseShooterIOSim implements BaseShooterIO {
    private final FlywheelSim flywheelSim;
    private final PIDController flywheelController;
    private final SimpleMotorFeedforward flywheelFeedforward;

    private double flywheelSetpointRPM = 0.0;
    private double flywheelAppliedVolts = 0.0;

    public BaseShooterIOSim(ShooterConstants.ShooterConfig config) {
        var flywheelPlant = LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(2),
                0.5 * 0.5 * ShooterConstants.flywheelRadius.in(Meters) * ShooterConstants.flywheelRadius.in(Meters),
                1.0);

        flywheelSim = new FlywheelSim(flywheelPlant, DCMotor.getKrakenX60Foc(2));

        flywheelController = new PIDController(0.001, 0.0005, 0.0);

        double flywheelFreeSpeedRPM = DCMotor.getKrakenX60Foc(2).freeSpeedRadPerSec * 60.0 / (2.0 * Math.PI);
        flywheelFeedforward = new SimpleMotorFeedforward(0.0, 12.0 / flywheelFreeSpeedRPM, 0.0);
    }

    @Override
    public void updateInputs(BaseShooterIOInputs inputs) {
        double flywheelFF = flywheelFeedforward.calculate(flywheelSetpointRPM);
        double flywheelFB = flywheelController.calculate(flywheelSim.getAngularVelocityRPM(), flywheelSetpointRPM);
        flywheelAppliedVolts = MathUtil.clamp(flywheelFF + flywheelFB, -12.0, 12.0);

        flywheelSim.setInputVoltage(flywheelAppliedVolts);

        flywheelSim.update(Robot.defaultPeriodSecs);

        inputs.flywheelVelocity = RPM.of(flywheelSim.getAngularVelocityRPM());
        inputs.flywheelAppliedVoltage = Volts.of(flywheelAppliedVolts);
        inputs.flywheelCurrent = Amps.of(flywheelSim.getCurrentDrawAmps());
        inputs.flywheelTemp = Celsius.of(25.0);
    }

    @Override
    public void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void stop() {
        flywheelSetpointRPM = 0.0;
        flywheelAppliedVolts = 0.0;
        flywheelSim.setInputVoltage(0.0);
    }
}
