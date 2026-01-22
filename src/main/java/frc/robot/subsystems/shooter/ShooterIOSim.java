package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(Constants.ShooterConstants.motorCount),
                    Constants.ShooterConstants.MOI,
                    Constants.ShooterConstants.gearRatio),
            DCMotor.getKrakenX60(Constants.ShooterConstants.motorCount),
            Constants.ShooterConstants.gearRatio);

    private double appliedVolts = 0.0;

    public ShooterIOSim() {}

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        flywheelSim.update(0.020); // 20ms update rate

        inputs.shooterMotorConnected = true;
        inputs.shooterPosition = inputs.shooterPosition.plus(
                Radians.of(flywheelSim.getAngularVelocity().in(RadiansPerSecond) * 0.020));
        inputs.shooterVelocity = flywheelSim.getAngularVelocity();
        inputs.shooterAppliedVolts = Volts.of(appliedVolts);
        inputs.shooterCurrent = Amps.of(flywheelSim.getCurrentDrawAmps());
    }

    @Override
    public void setVoltage(Voltage volts) {
        appliedVolts = volts.in(Volts);
        flywheelSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setVelocity(AngularVelocity velocityRadPerSec) {
        flywheelSim.setAngularVelocity(velocityRadPerSec.in(RadiansPerSecond));
    }

    @Override
    public void stop() {
        setVoltage(Volts.of(0.0));
    }
}
