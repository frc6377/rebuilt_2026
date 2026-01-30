// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {
    // Simulation models
    private final FlywheelSim leftFlywheelSim;
    private final FlywheelSim rightFlywheelSim;
    private final SingleJointedArmSim hoodSim;

    // PID Controllers for simulation
    private final PIDController leftFlywheelController;
    private final PIDController rightFlywheelController;
    private final PIDController hoodController;

    // Setpoints
    private double leftFlywheelSetpointRPM = 0.0;
    private double rightFlywheelSetpointRPM = 0.0;
    private double hoodSetpointDegrees = 0.0;

    // Applied voltages
    private double leftFlywheelAppliedVolts = 0.0;
    private double rightFlywheelAppliedVolts = 0.0;
    private double hoodAppliedVolts = 0.0;

    public ShooterIOSim() {
        // Create flywheel simulations (Kraken X60 motors)
        leftFlywheelSim = new FlywheelSim(DCMotor.getKrakenX60(1), 1.0, 0.004);
        rightFlywheelSim = new FlywheelSim(DCMotor.getKrakenX60(1), 1.0, 0.004);

        // Create hood simulation (Kraken x44 motor, single jointed arm)
        hoodSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1), // Kraken x44 approximated with Kraken X60
                ShooterConstants.hoodGearRatio,
                0.1, // MOI (kg*m^2) - adjust based on actual hood mass
                0.3, // Arm length (m) - adjust based on actual hood geometry
                ShooterConstants.minHoodAngle.in(Radians),
                ShooterConstants.maxHoodAngle.in(Radians),
                true, // Simulate gravity
                ShooterConstants.minHoodAngle.in(Radians) // Starting angle
                );

        // Create PID controllers
        leftFlywheelController =
                new PIDController(ShooterConstants.defaultFlywheelKP, ShooterConstants.defaultFlywheelKI, ShooterConstants.defaultFlywheelKD);
        rightFlywheelController =
                new PIDController(ShooterConstants.defaultFlywheelKP, ShooterConstants.defaultFlywheelKI, ShooterConstants.defaultFlywheelKD);
        hoodController = new PIDController(ShooterConstants.defaultHoodKP, ShooterConstants.defaultHoodKI, ShooterConstants.defaultHoodKD);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // PID controller for flywheel velocity (simulation)
        leftFlywheelAppliedVolts = MathUtil.clamp(
                leftFlywheelController.calculate(
                        leftFlywheelSim.getAngularVelocityRPM(), leftFlywheelSetpointRPM),
                -12.0,
                12.0);
        leftFlywheelSim.setInputVoltage(leftFlywheelAppliedVolts);

        rightFlywheelAppliedVolts = MathUtil.clamp(
                rightFlywheelController.calculate(
                        rightFlywheelSim.getAngularVelocityRPM(), rightFlywheelSetpointRPM),
                -12.0,
                12.0);
        rightFlywheelSim.setInputVoltage(rightFlywheelAppliedVolts);

        // PID controller for hood position (simulation)
        hoodAppliedVolts = MathUtil.clamp(
                hoodController.calculate(Math.toDegrees(hoodSim.getAngleRads()), hoodSetpointDegrees), -12.0, 12.0);
        hoodSim.setInputVoltage(hoodAppliedVolts);

        // Update flywheel simulations
        leftFlywheelSim.update(0.02); // 20ms period
        rightFlywheelSim.update(0.02);
        hoodSim.update(0.02);

        // Update inputs
        inputs.leftFlywheelVelocityRPM = leftFlywheelSim.getAngularVelocityRPM();
        inputs.leftFlywheelAppliedVolts = leftFlywheelAppliedVolts;
        inputs.leftFlywheelCurrentAmps = leftFlywheelSim.getCurrentDrawAmps();
        inputs.leftFlywheelTempCelsius = 25.0; // Simulated temp

        inputs.rightFlywheelVelocityRPM = rightFlywheelSim.getAngularVelocityRPM();
        inputs.rightFlywheelAppliedVolts = rightFlywheelAppliedVolts;
        inputs.rightFlywheelCurrentAmps = rightFlywheelSim.getCurrentDrawAmps();
        inputs.rightFlywheelTempCelsius = 25.0; // Simulated temp

        inputs.hoodAngleDegrees = Math.toDegrees(hoodSim.getAngleRads());
        inputs.hoodAppliedVolts = hoodAppliedVolts;
        inputs.hoodCurrentAmps = hoodSim.getCurrentDrawAmps();
        inputs.hoodTempCelsius = 25.0; // Simulated temp
    }

    @Override
    public void setLeftFlywheelVelocity(AngularVelocity velocity) {
        leftFlywheelSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void setRightFlywheelVelocity(AngularVelocity velocity) {
        rightFlywheelSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void setHoodAngle(Angle angle) {
        hoodSetpointDegrees = MathUtil.clamp(
                angle.in(Degrees), ShooterConstants.minHoodAngle.in(Degrees), ShooterConstants.maxHoodAngle.in(Degrees));
    }

    @Override
    public void stop() {
        leftFlywheelSetpointRPM = 0.0;
        rightFlywheelSetpointRPM = 0.0;
        leftFlywheelAppliedVolts = 0.0;
        rightFlywheelAppliedVolts = 0.0;
        // Stop hood control by setting setpoint to current angle and removing voltage
        hoodSetpointDegrees = Math.toDegrees(hoodSim.getAngleRads());
        hoodAppliedVolts = 0.0;
        hoodSim.setInputVoltage(0.0);
    }
}
