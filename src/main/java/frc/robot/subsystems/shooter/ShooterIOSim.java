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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {
    // Simulation models
    private final FlywheelSim leftFlywheelSim;
    private final FlywheelSim rightFlywheelSim;
    private final SingleJointedArmSim hoodSim;

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
                Math.toRadians(ShooterConstants.minHoodAngleDegrees),
                Math.toRadians(ShooterConstants.maxHoodAngleDegrees),
                true, // Simulate gravity
                Math.toRadians(ShooterConstants.minHoodAngleDegrees) // Starting angle
                );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update flywheel simulations
        leftFlywheelSim.update(0.02); // 20ms period
        rightFlywheelSim.update(0.02);
        hoodSim.update(0.02);

        // Simple P controller for flywheel velocity (simulation)
        double leftError = leftFlywheelSetpointRPM - (leftFlywheelSim.getAngularVelocityRPM());
        leftFlywheelAppliedVolts = MathUtil.clamp(leftError * 0.001, -12.0, 12.0);
        leftFlywheelSim.setInputVoltage(leftFlywheelAppliedVolts);

        double rightError = rightFlywheelSetpointRPM - (rightFlywheelSim.getAngularVelocityRPM());
        rightFlywheelAppliedVolts = MathUtil.clamp(rightError * 0.001, -12.0, 12.0);
        rightFlywheelSim.setInputVoltage(rightFlywheelAppliedVolts);

        // Simple P controller for hood position (simulation)
        double hoodError = Math.toRadians(hoodSetpointDegrees) - hoodSim.getAngleRads();
        hoodAppliedVolts = MathUtil.clamp(hoodError * 10.0, -12.0, 12.0);
        hoodSim.setInputVoltage(hoodAppliedVolts);

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
    public void setLeftFlywheelVelocity(double velocityRPM) {
        leftFlywheelSetpointRPM = velocityRPM;
    }

    @Override
    public void setRightFlywheelVelocity(double velocityRPM) {
        rightFlywheelSetpointRPM = velocityRPM;
    }

    @Override
    public void setHoodAngle(double angleDegrees) {
        hoodSetpointDegrees = MathUtil.clamp(
                angleDegrees, ShooterConstants.minHoodAngleDegrees, ShooterConstants.maxHoodAngleDegrees);
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
