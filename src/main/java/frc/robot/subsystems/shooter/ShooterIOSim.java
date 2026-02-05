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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class ShooterIOSim implements ShooterIO {
    // Motor models
    private static final DCMotor FLYWHEEL_MOTOR = DCMotor.getKrakenX60Foc(1); // Kraken X60 for flywheels
    private static final DCMotor SPIN_MOTOR = DCMotor.getKrakenX44Foc(1); // Kraken X44 for spin adjustment

    // Flywheel physical properties
    // MOI = 0.5 * m * r^2 for solid cylinder
    // Assuming ~0.5kg flywheel with 2" (0.0508m) radius
    private static final double FLYWHEEL_MASS_KG = 0.5;
    private static final double FLYWHEEL_RADIUS_M = ShooterConstants.flywheelRadius.in(Meters);
    private static final double FLYWHEEL_MOI = 0.5 * FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M;
    private static final double FLYWHEEL_GEARING = 1.0; // Direct drive

    // Spin motor physical properties (smaller, lighter roller)
    private static final double SPIN_MASS_KG = 0.2;
    private static final double SPIN_RADIUS_M = 0.025; // 1 inch radius
    private static final double SPIN_MOI = 0.5 * SPIN_MASS_KG * SPIN_RADIUS_M * SPIN_RADIUS_M;
    private static final double SPIN_GEARING = 1.0;

    // Simulation models - Flywheels
    private final FlywheelSim leftFlywheelSim;
    private final FlywheelSim rightFlywheelSim;

    // Simulation models - Spin motors
    private final FlywheelSim leftSpinSim;
    private final FlywheelSim rightSpinSim;

    // PID Controllers for flywheels (velocity control in RPM)
    private final PIDController leftFlywheelController;
    private final PIDController rightFlywheelController;

    // PID Controllers for spin motors
    private final PIDController leftSpinController;
    private final PIDController rightSpinController;

    // Feedforward for better velocity tracking
    private final SimpleMotorFeedforward flywheelFeedforward;
    private final SimpleMotorFeedforward spinFeedforward;

    // Flywheel setpoints
    private double leftFlywheelSetpointRPM = 0.0;
    private double rightFlywheelSetpointRPM = 0.0;

    // Spin motor setpoints
    private double leftSpinSetpointRPM = 0.0;
    private double rightSpinSetpointRPM = 0.0;

    // Applied voltages
    private double leftFlywheelAppliedVolts = 0.0;
    private double rightFlywheelAppliedVolts = 0.0;
    private double leftSpinAppliedVolts = 0.0;
    private double rightSpinAppliedVolts = 0.0;

    // Temperature simulation (simple thermal model)
    private double leftMotorTempCelsius = 25.0;
    private double rightMotorTempCelsius = 25.0;
    private double leftSpinTempCelsius = 25.0;
    private double rightSpinTempCelsius = 25.0;
    private static final double AMBIENT_TEMP = 25.0;
    private static final double THERMAL_RESISTANCE = 0.5; // °C/W - how fast motor heats up
    private static final double THERMAL_TIME_CONSTANT = 30.0; // seconds - thermal mass

    public ShooterIOSim() {
        // Create flywheel plant using proper physics
        var flywheelPlant = LinearSystemId.createFlywheelSystem(FLYWHEEL_MOTOR, FLYWHEEL_MOI, FLYWHEEL_GEARING);
        var spinPlant = LinearSystemId.createFlywheelSystem(SPIN_MOTOR, SPIN_MOI, SPIN_GEARING);

        leftFlywheelSim = new FlywheelSim(flywheelPlant, FLYWHEEL_MOTOR);
        rightFlywheelSim = new FlywheelSim(flywheelPlant, FLYWHEEL_MOTOR);
        leftSpinSim = new FlywheelSim(spinPlant, SPIN_MOTOR);
        rightSpinSim = new FlywheelSim(spinPlant, SPIN_MOTOR);

        // PID controllers tuned for simulation
        leftFlywheelController = new PIDController(0.001, 0.0005, 0.0);
        rightFlywheelController = new PIDController(0.001, 0.0005, 0.0);
        leftSpinController = new PIDController(0.001, 0.0005, 0.0);
        rightSpinController = new PIDController(0.001, 0.0005, 0.0);

        // Feedforward based on actual motor model
        double flywheelFreeSpeedRPM = FLYWHEEL_MOTOR.freeSpeedRadPerSec * 60.0 / (2.0 * Math.PI);
        double flywheelKV = 12.0 / flywheelFreeSpeedRPM;
        flywheelFeedforward = new SimpleMotorFeedforward(0.0, flywheelKV, 0.0);

        double spinFreeSpeedRPM = SPIN_MOTOR.freeSpeedRadPerSec * 60.0 / (2.0 * Math.PI);
        double spinKV = 12.0 / spinFreeSpeedRPM;
        spinFeedforward = new SimpleMotorFeedforward(0.0, spinKV, 0.0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Calculate control output for flywheels using feedforward + feedback
        double leftFF = flywheelFeedforward.calculate(leftFlywheelSetpointRPM);
        double leftFB =
                leftFlywheelController.calculate(leftFlywheelSim.getAngularVelocityRPM(), leftFlywheelSetpointRPM);
        leftFlywheelAppliedVolts = MathUtil.clamp(leftFF + leftFB, -12.0, 12.0);

        double rightFF = flywheelFeedforward.calculate(rightFlywheelSetpointRPM);
        double rightFB =
                rightFlywheelController.calculate(rightFlywheelSim.getAngularVelocityRPM(), rightFlywheelSetpointRPM);
        rightFlywheelAppliedVolts = MathUtil.clamp(rightFF + rightFB, -12.0, 12.0);

        // Calculate control output for spin motors
        double leftSpinFF = spinFeedforward.calculate(leftSpinSetpointRPM);
        double leftSpinFB = leftSpinController.calculate(leftSpinSim.getAngularVelocityRPM(), leftSpinSetpointRPM);
        leftSpinAppliedVolts = MathUtil.clamp(leftSpinFF + leftSpinFB, -12.0, 12.0);

        double rightSpinFF = spinFeedforward.calculate(rightSpinSetpointRPM);
        double rightSpinFB = rightSpinController.calculate(rightSpinSim.getAngularVelocityRPM(), rightSpinSetpointRPM);
        rightSpinAppliedVolts = MathUtil.clamp(rightSpinFF + rightSpinFB, -12.0, 12.0);

        // Apply voltage to simulations
        leftFlywheelSim.setInputVoltage(leftFlywheelAppliedVolts);
        rightFlywheelSim.setInputVoltage(rightFlywheelAppliedVolts);
        leftSpinSim.setInputVoltage(leftSpinAppliedVolts);
        rightSpinSim.setInputVoltage(rightSpinAppliedVolts);

        // Update all simulations
        leftFlywheelSim.update(0.02); // 20ms period
        rightFlywheelSim.update(0.02);
        leftSpinSim.update(0.02);
        rightSpinSim.update(0.02);

        // Update temperature simulation (simple first-order thermal model)
        double leftPowerDissipated = leftFlywheelSim.getCurrentDrawAmps() * Math.abs(leftFlywheelAppliedVolts) * 0.1;
        double rightPowerDissipated = rightFlywheelSim.getCurrentDrawAmps() * Math.abs(rightFlywheelAppliedVolts) * 0.1;
        double leftSpinPowerDissipated = leftSpinSim.getCurrentDrawAmps() * Math.abs(leftSpinAppliedVolts) * 0.1;
        double rightSpinPowerDissipated = rightSpinSim.getCurrentDrawAmps() * Math.abs(rightSpinAppliedVolts) * 0.1;

        double leftEquilibriumTemp = AMBIENT_TEMP + leftPowerDissipated * THERMAL_RESISTANCE;
        double rightEquilibriumTemp = AMBIENT_TEMP + rightPowerDissipated * THERMAL_RESISTANCE;
        double leftSpinEquilibriumTemp = AMBIENT_TEMP + leftSpinPowerDissipated * THERMAL_RESISTANCE;
        double rightSpinEquilibriumTemp = AMBIENT_TEMP + rightSpinPowerDissipated * THERMAL_RESISTANCE;

        double dt = Robot.defaultPeriodSecs;
        leftMotorTempCelsius += (leftEquilibriumTemp - leftMotorTempCelsius) * dt / THERMAL_TIME_CONSTANT;
        rightMotorTempCelsius += (rightEquilibriumTemp - rightMotorTempCelsius) * dt / THERMAL_TIME_CONSTANT;
        leftSpinTempCelsius += (leftSpinEquilibriumTemp - leftSpinTempCelsius) * dt / THERMAL_TIME_CONSTANT;
        rightSpinTempCelsius += (rightSpinEquilibriumTemp - rightSpinTempCelsius) * dt / THERMAL_TIME_CONSTANT;

        // Update flywheel inputs
        inputs.leftFlywheelVelocity = RPM.of(leftFlywheelSim.getAngularVelocityRPM());
        inputs.leftFlywheelAppliedVoltage = Volts.of(leftFlywheelAppliedVolts);
        inputs.leftFlywheelCurrent = Amps.of(leftFlywheelSim.getCurrentDrawAmps());
        inputs.leftFlywheelTemp = Celsius.of(leftMotorTempCelsius);

        inputs.rightFlywheelVelocity = RPM.of(rightFlywheelSim.getAngularVelocityRPM());
        inputs.rightFlywheelAppliedVoltage = Volts.of(rightFlywheelAppliedVolts);
        inputs.rightFlywheelCurrent = Amps.of(rightFlywheelSim.getCurrentDrawAmps());
        inputs.rightFlywheelTemp = Celsius.of(rightMotorTempCelsius);

        // Update spin motor inputs
        inputs.leftSpinVelocity = RPM.of(leftSpinSim.getAngularVelocityRPM());
        inputs.leftSpinAppliedVoltage = Volts.of(leftSpinAppliedVolts);
        inputs.leftSpinCurrent = Amps.of(leftSpinSim.getCurrentDrawAmps());
        inputs.leftSpinTemp = Celsius.of(leftSpinTempCelsius);

        inputs.rightSpinVelocity = RPM.of(rightSpinSim.getAngularVelocityRPM());
        inputs.rightSpinAppliedVoltage = Volts.of(rightSpinAppliedVolts);
        inputs.rightSpinCurrent = Amps.of(rightSpinSim.getCurrentDrawAmps());
        inputs.rightSpinTemp = Celsius.of(rightSpinTempCelsius);
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
    public void setLeftSpinVelocity(AngularVelocity velocity) {
        leftSpinSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void setRightSpinVelocity(AngularVelocity velocity) {
        rightSpinSetpointRPM = velocity.in(RPM);
    }

    @Override
    public void stop() {
        leftFlywheelSetpointRPM = 0.0;
        rightFlywheelSetpointRPM = 0.0;
        leftSpinSetpointRPM = 0.0;
        rightSpinSetpointRPM = 0.0;
        leftFlywheelAppliedVolts = 0.0;
        rightFlywheelAppliedVolts = 0.0;
        leftSpinAppliedVolts = 0.0;
        rightSpinAppliedVolts = 0.0;
        leftFlywheelSim.setInputVoltage(0.0);
        rightFlywheelSim.setInputVoltage(0.0);
        leftSpinSim.setInputVoltage(0.0);
        rightSpinSim.setInputVoltage(0.0);
    }

    /**
     * Get the current launch velocity based on flywheel surface speed. v_launch = omega * r * efficiency
     *
     * @return Launch velocity in meters per second
     */
    public double getLaunchVelocityMPS() {
        double avgRPM = (leftFlywheelSim.getAngularVelocityRPM() + rightFlywheelSim.getAngularVelocityRPM()) / 2.0;
        double omegaRadPerSec = avgRPM * 2.0 * Math.PI / 60.0;
        return omegaRadPerSec * FLYWHEEL_RADIUS_M * ShooterConstants.launchEfficiency;
    }
}
