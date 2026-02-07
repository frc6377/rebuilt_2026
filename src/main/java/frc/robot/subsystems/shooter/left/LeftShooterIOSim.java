package frc.robot.subsystems.shooter.left;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterConstants;

public class LeftShooterIOSim implements LeftShooterIO {
    private final FlywheelSim flywheelSim;
    private final FlywheelSim spinSim;
    private final PIDController flywheelController;
    private final PIDController spinController;
    private final SimpleMotorFeedforward flywheelFeedforward;
    private final SimpleMotorFeedforward spinFeedforward;

    private double flywheelSetpointRPM = 0.0;
    private double spinSetpointRPM = 0.0;
    private double flywheelAppliedVolts = 0.0;
    private double spinAppliedVolts = 0.0;

    public LeftShooterIOSim() {
        var flywheelPlant = LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(2),
                0.5 * 0.5 * ShooterConstants.flywheelRadius.in(Meters) * ShooterConstants.flywheelRadius.in(Meters),
                1.0);
        var spinPlant = LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44Foc(1), 0.5 * 0.2 * 0.025 * 0.025, 1.0);

        flywheelSim = new FlywheelSim(flywheelPlant, DCMotor.getKrakenX60Foc(2));
        spinSim = new FlywheelSim(spinPlant, DCMotor.getKrakenX44Foc(1));

        flywheelController = new PIDController(0.001, 0.0005, 0.0);
        spinController = new PIDController(0.001, 0.0005, 0.0);

        double flywheelFreeSpeedRPM = DCMotor.getKrakenX60Foc(2).freeSpeedRadPerSec * 60.0 / (2.0 * Math.PI);
        flywheelFeedforward = new SimpleMotorFeedforward(0.0, 12.0 / flywheelFreeSpeedRPM, 0.0);

        double spinFreeSpeedRPM = DCMotor.getKrakenX44Foc(1).freeSpeedRadPerSec * 60.0 / (2.0 * Math.PI);
        spinFeedforward = new SimpleMotorFeedforward(0.0, 12.0 / spinFreeSpeedRPM, 0.0);
    }

    @Override
    public void updateInputs(LeftShooterIOInputs inputs) {
        double flywheelFF = flywheelFeedforward.calculate(flywheelSetpointRPM);
        double flywheelFB = flywheelController.calculate(flywheelSim.getAngularVelocityRPM(), flywheelSetpointRPM);
        flywheelAppliedVolts = MathUtil.clamp(flywheelFF + flywheelFB, -12.0, 12.0);

        double spinFF = spinFeedforward.calculate(spinSetpointRPM);
        double spinFB = spinController.calculate(spinSim.getAngularVelocityRPM(), spinSetpointRPM);
        spinAppliedVolts = MathUtil.clamp(spinFF + spinFB, -12.0, 12.0);

        flywheelSim.setInputVoltage(flywheelAppliedVolts);
        spinSim.setInputVoltage(spinAppliedVolts);

        flywheelSim.update(Robot.defaultPeriodSecs);
        spinSim.update(Robot.defaultPeriodSecs);

        inputs.flywheelVelocity = RPM.of(flywheelSim.getAngularVelocityRPM());
        inputs.flywheelAppliedVoltage = Volts.of(flywheelAppliedVolts);
        inputs.flywheelCurrent = Amps.of(flywheelSim.getCurrentDrawAmps());
        inputs.flywheelTemp = Celsius.of(25.0);

        inputs.spinVelocity = RPM.of(spinSim.getAngularVelocityRPM());
        inputs.spinAppliedVoltage = Volts.of(spinAppliedVolts);
        inputs.spinCurrent = Amps.of(spinSim.getCurrentDrawAmps());
        inputs.spinTemp = Celsius.of(25.0);
    }

    @Override
    public void setFlywheelVelocity(AngularVelocity velocity) {
        flywheelSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void setSpinVelocity(AngularVelocity velocity) {
        spinSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void stop() {
        flywheelSetpointRPM = 0.0;
        spinSetpointRPM = 0.0;
        flywheelAppliedVolts = 0.0;
        spinAppliedVolts = 0.0;
        flywheelSim.setInputVoltage(0.0);
        spinSim.setInputVoltage(0.0);
    }
}
