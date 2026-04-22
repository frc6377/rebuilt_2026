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
import org.jetbrains.annotations.NotNull;

public class BaseShooterIOSim implements BaseShooterIO {
    private final @NotNull FlywheelSim flywheelSim;
    private final @NotNull PIDController flywheelController;
    private final @NotNull SimpleMotorFeedforward flywheelFeedforward;

    private double flywheelSetpointRPM = 0.0;
    private double flywheelAppliedVolts = 0.0;

    public BaseShooterIOSim(ShooterConstants.ShooterConfig config) {
        var flywheelPlant = LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(2),
                0.5 * 0.5 * ShooterConstants.flywheelRadius.in(Meters) * ShooterConstants.flywheelRadius.in(Meters),
                1.0);

        this.flywheelSim = new FlywheelSim(flywheelPlant, DCMotor.getKrakenX60Foc(2));

        this.flywheelController = new PIDController(0.001, 0.0005, 0.0);

        double flywheelFreeSpeedRPM = DCMotor.getKrakenX60Foc(2).freeSpeedRadPerSec * 60.0 / (2.0 * Math.PI);
        this.flywheelFeedforward = new SimpleMotorFeedforward(0.0, 12.0 / flywheelFreeSpeedRPM, 0.0);
    }

    @Override
    public void updateInputs(@NotNull BaseShooterIOInputs inputs) {
        double flywheelFF = this.flywheelFeedforward.calculate(this.flywheelSetpointRPM);
        double flywheelFB = this.flywheelController.calculate(this.flywheelSim.getAngularVelocityRPM(), this.flywheelSetpointRPM);
        this.flywheelAppliedVolts = MathUtil.clamp(flywheelFF + flywheelFB, -12.0, 12.0);

        this.flywheelSim.setInputVoltage(this.flywheelAppliedVolts);

        this.flywheelSim.update(Robot.defaultPeriodSecs);

        inputs.flywheelVelocity = RPM.of(this.flywheelSim.getAngularVelocityRPM());
        inputs.flywheelAppliedVoltage = Volts.of(this.flywheelAppliedVolts);
        inputs.flywheelCurrent = Amps.of(this.flywheelSim.getCurrentDrawAmps());
        inputs.flywheelTemp = Celsius.of(25.0);
    }

    @Override
    public void setFlywheelVelocity(@NotNull AngularVelocity velocity) {
        this.flywheelSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void stop() {
        this.flywheelSetpointRPM = 0.0;
        this.flywheelAppliedVolts = 0.0;
        this.flywheelSim.setInputVoltage(0.0);
    }
}
