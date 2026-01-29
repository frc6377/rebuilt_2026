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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * Hooded shooter subsystem with dual-sided IO layer.
 * Features independent Kraken X60 motors for redundancy and limp-mode operation.
 */
public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Tunable PID constants
    private final LoggedDashboardNumber flywheelKP = new LoggedDashboardNumber("Shooter/FlywheelKP", 0.1);
    private final LoggedDashboardNumber flywheelKI = new LoggedDashboardNumber("Shooter/FlywheelKI", 0.0);
    private final LoggedDashboardNumber flywheelKD = new LoggedDashboardNumber("Shooter/FlywheelKD", 0.0);
    private final LoggedDashboardNumber flywheelKV = new LoggedDashboardNumber("Shooter/FlywheelKV", 0.12);
    private final LoggedDashboardNumber flywheelKS = new LoggedDashboardNumber("Shooter/FlywheelKS", 0.0);

    private final LoggedDashboardNumber hoodKP = new LoggedDashboardNumber("Shooter/HoodKP", 10.0);
    private final LoggedDashboardNumber hoodKI = new LoggedDashboardNumber("Shooter/HoodKI", 0.0);
    private final LoggedDashboardNumber hoodKD = new LoggedDashboardNumber("Shooter/HoodKD", 0.5);

    // Interpolation map for distance-based shooting
    // Maps distance (meters) to shooter settings (velocity RPM, hood angle degrees)
    private final InterpolatingDoubleTreeMap distanceToVelocityMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceToAngleMap = new InterpolatingDoubleTreeMap();

    // Setpoints
    private double leftFlywheelSetpoint = 0.0;
    private double rightFlywheelSetpoint = 0.0;
    private double hoodAngleSetpoint = 0.0;

    // Limp mode state
    private boolean leftFlywheelFailed = false;
    private boolean rightFlywheelFailed = false;

    public Shooter(ShooterIO io) {
        this.io = io;
        configureShotMaps();
    }

    /**
     * Configure distance-based shot interpolation maps.
     * These values should be tuned based on actual robot performance.
     */
    private void configureShotMaps() {
        // Distance in meters -> Flywheel velocity in RPM
        distanceToVelocityMap.put(1.0, 2000.0); // 1m -> 2000 RPM
        distanceToVelocityMap.put(2.0, 2500.0); // 2m -> 2500 RPM
        distanceToVelocityMap.put(3.0, 3000.0); // 3m -> 3000 RPM
        distanceToVelocityMap.put(4.0, 3500.0); // 4m -> 3500 RPM
        distanceToVelocityMap.put(5.0, 4000.0); // 5m -> 4000 RPM

        // Distance in meters -> Hood angle in degrees
        distanceToAngleMap.put(1.0, 15.0); // 1m -> 15°
        distanceToAngleMap.put(2.0, 20.0); // 2m -> 20°
        distanceToAngleMap.put(3.0, 25.0); // 3m -> 25°
        distanceToAngleMap.put(4.0, 30.0); // 4m -> 30°
        distanceToAngleMap.put(5.0, 35.0); // 5m -> 35°
    }

    @Override
    public void periodic() {
        // Update and log inputs
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Check for motor failures and enable limp mode if needed
        detectMotorFailures();

        // Log additional state
        Logger.recordOutput("Shooter/LeftFlywheelSetpoint", leftFlywheelSetpoint);
        Logger.recordOutput("Shooter/RightFlywheelSetpoint", rightFlywheelSetpoint);
        Logger.recordOutput("Shooter/HoodAngleSetpoint", hoodAngleSetpoint);
        Logger.recordOutput("Shooter/LeftFlywheelFailed", leftFlywheelFailed);
        Logger.recordOutput("Shooter/RightFlywheelFailed", rightFlywheelFailed);
        Logger.recordOutput("Shooter/InLimpMode", isInLimpMode());
    }

    /**
     * Set velocity for both flywheels independently.
     * This allows for limp-mode operation if one side fails.
     *
     * @param leftRPM Target velocity for left flywheel in RPM
     * @param rightRPM Target velocity for right flywheel in RPM
     */
    public void setFlywheelVelocities(double leftRPM, double rightRPM) {
        leftFlywheelSetpoint = leftRPM;
        rightFlywheelSetpoint = rightRPM;

        // Only send commands to non-failed motors
        if (!leftFlywheelFailed) {
            io.setLeftFlywheelVelocity(leftRPM);
        } else {
            io.setLeftFlywheelVelocity(0.0); // Stop failed motor
        }

        if (!rightFlywheelFailed) {
            io.setRightFlywheelVelocity(rightRPM);
        } else {
            io.setRightFlywheelVelocity(0.0); // Stop failed motor
        }
    }

    /**
     * Set velocity for both flywheels to the same value.
     * Uses independent control to prevent "fighting" between motors.
     *
     * @param velocityRPM Target velocity in RPM
     */
    public void setFlywheelVelocity(double velocityRPM) {
        setFlywheelVelocities(velocityRPM, velocityRPM);
    }

    /**
     * Set hood angle in degrees.
     *
     * @param angleDegrees Target angle in degrees
     */
    public void setHoodAngle(double angleDegrees) {
        hoodAngleSetpoint = angleDegrees;
        io.setHoodAngle(angleDegrees);
    }

    /**
     * Prepare shooter for a shot at a specific distance.
     * Automatically calculates and sets the appropriate flywheel velocity and hood angle.
     *
     * @param distanceMeters Distance to target in meters
     */
    public void prepareForDistance(double distanceMeters) {
        double targetVelocity = distanceToVelocityMap.get(distanceMeters);
        double targetAngle = distanceToAngleMap.get(distanceMeters);

        setFlywheelVelocity(targetVelocity);
        setHoodAngle(targetAngle);

        Logger.recordOutput("Shooter/AutoAim/Distance", distanceMeters);
        Logger.recordOutput("Shooter/AutoAim/CalculatedVelocity", targetVelocity);
        Logger.recordOutput("Shooter/AutoAim/CalculatedAngle", targetAngle);
    }

    /**
     * Stop all shooter motors.
     */
    public void stop() {
        leftFlywheelSetpoint = 0.0;
        rightFlywheelSetpoint = 0.0;
        io.stop();
    }

    /**
     * Get current left flywheel velocity.
     *
     * @return Velocity in RPM
     */
    public double getLeftFlywheelVelocity() {
        return inputs.leftFlywheelVelocityRPM;
    }

    /**
     * Get current right flywheel velocity.
     *
     * @return Velocity in RPM
     */
    public double getRightFlywheelVelocity() {
        return inputs.rightFlywheelVelocityRPM;
    }

    /**
     * Get current hood angle.
     *
     * @return Angle in degrees
     */
    public double getHoodAngle() {
        return inputs.hoodAngleDegrees;
    }

    /**
     * Check if flywheels are at target velocity.
     *
     * @return true if both active flywheels are within tolerance
     */
    @AutoLogOutput(key = "Shooter/AtTargetVelocity")
    public boolean atTargetVelocity() {
        double toleranceRPM = ShooterConstants.flywheelVelocityTolerance.in(RotationsPerSecond) * 60.0;
        boolean leftAtTarget = leftFlywheelFailed
                || Math.abs(inputs.leftFlywheelVelocityRPM - leftFlywheelSetpoint) < toleranceRPM;
        boolean rightAtTarget = rightFlywheelFailed
                || Math.abs(inputs.rightFlywheelVelocityRPM - rightFlywheelSetpoint) < toleranceRPM;

        return leftAtTarget && rightAtTarget;
    }

    /**
     * Check if system is in limp mode (one or more motors failed).
     *
     * @return true if in limp mode
     */
    public boolean isInLimpMode() {
        return leftFlywheelFailed || rightFlywheelFailed;
    }

    /**
     * Detect motor failures based on velocity tracking.
     * A motor is considered failed if it cannot reach the setpoint or
     * if there's excessive difference between the two flywheels.
     */
    private void detectMotorFailures() {
        // Only check for failures when motors are supposed to be running
        if (leftFlywheelSetpoint > 100 || rightFlywheelSetpoint > 100) {
            // Check for excessive velocity difference (potential mechanical failure)
            double velocityDifference = Math.abs(inputs.leftFlywheelVelocityRPM - inputs.rightFlywheelVelocityRPM);
            double maxDifferenceRPM = ShooterConstants.maxVelocityDifference.in(RotationsPerSecond) * 60.0;

            if (velocityDifference > maxDifferenceRPM) {
                // Determine which motor is likely failed (the slower one)
                if (inputs.leftFlywheelVelocityRPM < inputs.rightFlywheelVelocityRPM) {
                    leftFlywheelFailed = true;
                    Logger.recordOutput("Shooter/Alert", "Left flywheel failure detected - entering limp mode");
                } else {
                    rightFlywheelFailed = true;
                    Logger.recordOutput("Shooter/Alert", "Right flywheel failure detected - entering limp mode");
                }
            }
        }
    }

    /**
     * Reset failure flags (for testing or after maintenance).
     */
    public void resetFailureFlags() {
        leftFlywheelFailed = false;
        rightFlywheelFailed = false;
    }

    // ========== Command Factory Methods ==========

    /**
     * Command to spin up flywheels to a target velocity.
     *
     * @param velocityRPM Target velocity in RPM
     * @return Command that runs the flywheels
     */
    public Command spinUpFlywheels(double velocityRPM) {
        return Commands.run(() -> setFlywheelVelocity(velocityRPM), this).withName("SpinUpFlywheels");
    }

    /**
     * Command to spin up flywheels with a dynamic velocity supplier.
     *
     * @param velocitySupplier Supplier for target velocity in RPM
     * @return Command that runs the flywheels
     */
    public Command spinUpFlywheels(DoubleSupplier velocitySupplier) {
        return Commands.run(() -> setFlywheelVelocity(velocitySupplier.getAsDouble()), this)
                .withName("SpinUpFlywheels");
    }

    /**
     * Command to set hood angle.
     *
     * @param angleDegrees Target angle in degrees
     * @return Command that sets the hood angle
     */
    public Command setHoodAngleCommand(double angleDegrees) {
        return Commands.runOnce(() -> setHoodAngle(angleDegrees), this).withName("SetHoodAngle");
    }

    /**
     * Command to set hood angle with a dynamic angle supplier.
     *
     * @param angleSupplier Supplier for target angle in degrees
     * @return Command that sets the hood angle
     */
    public Command setHoodAngleCommand(DoubleSupplier angleSupplier) {
        return Commands.run(() -> setHoodAngle(angleSupplier.getAsDouble()), this).withName("SetHoodAngle");
    }

    /**
     * Command to stop all shooter motors.
     *
     * @return Command that stops the shooter
     */
    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("StopShooter");
    }

    /**
     * Command to prepare shooter for a shot (spin up and set hood angle).
     *
     * @param velocityRPM Target flywheel velocity in RPM
     * @param angleDegrees Target hood angle in degrees
     * @return Command that prepares the shooter
     */
    public Command prepareShot(double velocityRPM, double angleDegrees) {
        return Commands.parallel(spinUpFlywheels(velocityRPM), setHoodAngleCommand(angleDegrees))
                .withName("PrepareShot");
    }

    /**
     * Command to prepare shooter for a shot at a specific distance.
     * Automatically calculates and sets appropriate velocity and angle.
     *
     * @param distanceMeters Distance to target in meters
     * @return Command that prepares the shooter for the distance
     */
    public Command prepareForDistanceCommand(double distanceMeters) {
        return Commands.runOnce(() -> prepareForDistance(distanceMeters), this).withName("PrepareForDistance");
    }

    /**
     * Command to prepare shooter for a shot at a dynamic distance.
     *
     * @param distanceSupplier Supplier for distance to target in meters
     * @return Command that prepares the shooter for the distance
     */
    public Command prepareForDistanceCommand(DoubleSupplier distanceSupplier) {
        return Commands.run(() -> prepareForDistance(distanceSupplier.getAsDouble()), this)
                .withName("PrepareForDistance");
    }

    /**
     * Command to wait until flywheels are at target velocity.
     *
     * @return Command that waits until ready
     */
    public Command waitUntilReady() {
        return Commands.waitUntil(this::atTargetVelocity).withName("WaitUntilReady");
    }

    /**
     * Command to prepare shooter and wait until ready.
     *
     * @param velocityRPM Target flywheel velocity in RPM
     * @param angleDegrees Target hood angle in degrees
     * @return Command that prepares and waits
     */
    public Command prepareShotAndWait(double velocityRPM, double angleDegrees) {
        return prepareShot(velocityRPM, angleDegrees).andThen(waitUntilReady()).withName("PrepareShotAndWait");
    }

    /**
     * Command to prepare shooter for a distance and wait until ready.
     *
     * @param distanceMeters Distance to target in meters
     * @return Command that prepares for distance and waits
     */
    public Command prepareForDistanceAndWait(double distanceMeters) {
        return prepareForDistanceCommand(distanceMeters).andThen(waitUntilReady()).withName("PrepareForDistanceAndWait");
    }

    // ========== Tunable PID Getters ==========

    public double getFlywheelKP() {
        return flywheelKP.get();
    }

    public double getFlywheelKI() {
        return flywheelKI.get();
    }

    public double getFlywheelKD() {
        return flywheelKD.get();
    }

    public double getFlywheelKV() {
        return flywheelKV.get();
    }

    public double getFlywheelKS() {
        return flywheelKS.get();
    }

    public double getHoodKP() {
        return hoodKP.get();
    }

    public double getHoodKI() {
        return hoodKI.get();
    }

    public double getHoodKD() {
        return hoodKD.get();
    }
}
