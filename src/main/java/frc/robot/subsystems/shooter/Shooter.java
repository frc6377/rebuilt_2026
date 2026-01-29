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

    // Hood enable flag
    private final LoggedDashboardNumber hoodEnabled = new LoggedDashboardNumber("Shooter/HoodEnabled", 1.0);

    // Tunable shot maps for distance-based shooting
    // Distance 1m
    private final LoggedDashboardNumber dist1m = new LoggedDashboardNumber("Shooter/ShotMap/Dist1m", 1.0);
    private final LoggedDashboardNumber vel1m = new LoggedDashboardNumber("Shooter/ShotMap/Vel1m", 2000.0);
    private final LoggedDashboardNumber angle1m = new LoggedDashboardNumber("Shooter/ShotMap/Angle1m", 15.0);
    // Distance 2m
    private final LoggedDashboardNumber dist2m = new LoggedDashboardNumber("Shooter/ShotMap/Dist2m", 2.0);
    private final LoggedDashboardNumber vel2m = new LoggedDashboardNumber("Shooter/ShotMap/Vel2m", 2500.0);
    private final LoggedDashboardNumber angle2m = new LoggedDashboardNumber("Shooter/ShotMap/Angle2m", 20.0);
    // Distance 3m
    private final LoggedDashboardNumber dist3m = new LoggedDashboardNumber("Shooter/ShotMap/Dist3m", 3.0);
    private final LoggedDashboardNumber vel3m = new LoggedDashboardNumber("Shooter/ShotMap/Vel3m", 3000.0);
    private final LoggedDashboardNumber angle3m = new LoggedDashboardNumber("Shooter/ShotMap/Angle3m", 25.0);
    // Distance 4m
    private final LoggedDashboardNumber dist4m = new LoggedDashboardNumber("Shooter/ShotMap/Dist4m", 4.0);
    private final LoggedDashboardNumber vel4m = new LoggedDashboardNumber("Shooter/ShotMap/Vel4m", 3500.0);
    private final LoggedDashboardNumber angle4m = new LoggedDashboardNumber("Shooter/ShotMap/Angle4m", 30.0);
    // Distance 5m
    private final LoggedDashboardNumber dist5m = new LoggedDashboardNumber("Shooter/ShotMap/Dist5m", 5.0);
    private final LoggedDashboardNumber vel5m = new LoggedDashboardNumber("Shooter/ShotMap/Vel5m", 4000.0);
    private final LoggedDashboardNumber angle5m = new LoggedDashboardNumber("Shooter/ShotMap/Angle5m", 35.0);

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
    }

    @Override
    public void periodic() {
        // Update shot maps from tunable values
        updateShotMaps();

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
     * Update distance-based shot interpolation maps from tunable values.
     * Called each periodic cycle to allow real-time tuning.
     */
    private void updateShotMaps() {
        distanceToVelocityMap.clear();
        distanceToAngleMap.clear();

        // Update velocity map
        distanceToVelocityMap.put(dist1m.get(), vel1m.get());
        distanceToVelocityMap.put(dist2m.get(), vel2m.get());
        distanceToVelocityMap.put(dist3m.get(), vel3m.get());
        distanceToVelocityMap.put(dist4m.get(), vel4m.get());
        distanceToVelocityMap.put(dist5m.get(), vel5m.get());

        // Update angle map
        distanceToAngleMap.put(dist1m.get(), angle1m.get());
        distanceToAngleMap.put(dist2m.get(), angle2m.get());
        distanceToAngleMap.put(dist3m.get(), angle3m.get());
        distanceToAngleMap.put(dist4m.get(), angle4m.get());
        distanceToAngleMap.put(dist5m.get(), angle5m.get());
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
     * Only sends commands if hood is enabled via NetworkTables.
     *
     * @param angleDegrees Target angle in degrees
     */
    public void setHoodAngle(double angleDegrees) {
        hoodAngleSetpoint = angleDegrees;
        if (isHoodEnabled()) {
            io.setHoodAngle(angleDegrees);
        }
    }

    /**
     * Check if hood is enabled.
     *
     * @return true if hood enabled flag is non-zero
     */
    public boolean isHoodEnabled() {
        return hoodEnabled.get() != 0.0;
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
     * A motor is considered failed if there's excessive difference between
     * the two flywheels that cannot be explained by different setpoints.
     */
    private void detectMotorFailures() {
        // Only check for failures when motors are supposed to be running
        if (leftFlywheelSetpoint > 100 || rightFlywheelSetpoint > 100) {
            // Calculate expected velocity difference based on setpoints
            double expectedDifference = Math.abs(leftFlywheelSetpoint - rightFlywheelSetpoint);

            // Calculate actual velocity difference
            double actualDifference = Math.abs(inputs.leftFlywheelVelocityRPM - inputs.rightFlywheelVelocityRPM);

            // Calculate unexplained difference (potential failure)
            double unexplainedDifference = Math.abs(actualDifference - expectedDifference);

            double maxDifferenceRPM = ShooterConstants.maxVelocityDifference.in(RotationsPerSecond) * 60.0;

            // Only trigger failure if the difference can't be explained by different setpoints
            if (unexplainedDifference > maxDifferenceRPM) {
                // Determine which motor is likely failed (the slower one relative to its setpoint)
                double leftError = Math.abs(inputs.leftFlywheelVelocityRPM - leftFlywheelSetpoint);
                double rightError = Math.abs(inputs.rightFlywheelVelocityRPM - rightFlywheelSetpoint);

                if (leftError > rightError) {
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
