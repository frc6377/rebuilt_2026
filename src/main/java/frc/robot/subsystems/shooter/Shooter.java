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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Shooter subsystem with dual-sided IO layer. Features independent Kraken X60 motors for redundancy and limp-mode
 * operation.
 */
public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    // Tunable shot map for distance-based shooting (velocity only)
    // Distance 1m
    private final LoggedNetworkNumber dist1m = new LoggedNetworkNumber("Shooter/ShotMap/Dist1m", 1.0);
    private final LoggedNetworkNumber vel1m = new LoggedNetworkNumber("Shooter/ShotMap/Vel1m", 2000.0);
    // Distance 2m
    private final LoggedNetworkNumber dist2m = new LoggedNetworkNumber("Shooter/ShotMap/Dist2m", 2.0);
    private final LoggedNetworkNumber vel2m = new LoggedNetworkNumber("Shooter/ShotMap/Vel2m", 2500.0);
    // Distance 3m
    private final LoggedNetworkNumber dist3m = new LoggedNetworkNumber("Shooter/ShotMap/Dist3m", 3.0);
    private final LoggedNetworkNumber vel3m = new LoggedNetworkNumber("Shooter/ShotMap/Vel3m", 3000.0);
    // Distance 4m
    private final LoggedNetworkNumber dist4m = new LoggedNetworkNumber("Shooter/ShotMap/Dist4m", 4.0);
    private final LoggedNetworkNumber vel4m = new LoggedNetworkNumber("Shooter/ShotMap/Vel4m", 3500.0);
    // Distance 5m
    private final LoggedNetworkNumber dist5m = new LoggedNetworkNumber("Shooter/ShotMap/Dist5m", 5.0);
    private final LoggedNetworkNumber vel5m = new LoggedNetworkNumber("Shooter/ShotMap/Vel5m", 4000.0);

    // Tunable spin ratio (spin motor velocity as fraction of main flywheel velocity)
    private final LoggedNetworkNumber spinRatio =
            new LoggedNetworkNumber("Shooter/SpinRatio", ShooterConstants.defaultSpinRatio);

    // Tunable PID gains for flywheel velocity control
    private final LoggedNetworkNumber flywheelKP =
            new LoggedNetworkNumber("Shooter/Flywheel/kP", ShooterConstants.defaultFlywheelKP);
    private final LoggedNetworkNumber flywheelKI =
            new LoggedNetworkNumber("Shooter/Flywheel/kI", ShooterConstants.defaultFlywheelKI);
    private final LoggedNetworkNumber flywheelKD =
            new LoggedNetworkNumber("Shooter/Flywheel/kD", ShooterConstants.defaultFlywheelKD);
    private final LoggedNetworkNumber flywheelKV =
            new LoggedNetworkNumber("Shooter/Flywheel/kV", ShooterConstants.defaultFlywheelKV);
    private final LoggedNetworkNumber flywheelKS =
            new LoggedNetworkNumber("Shooter/Flywheel/kS", ShooterConstants.defaultFlywheelKS);

    // Interpolation map for distance-based shooting
    // Maps distance (meters) to shooter velocity RPM
    private final InterpolatingDoubleTreeMap distanceToVelocityMap = new InterpolatingDoubleTreeMap();

    // Setpoints
    private double leftFlywheelSetpoint = 0.0;
    private double rightFlywheelSetpoint = 0.0;
    private double leftSpinSetpoint = 0.0;
    private double rightSpinSetpoint = 0.0;

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
        Logger.recordOutput("Shooter/LeftFlywheelEnabled", ShooterConstants.leftFlywheelEnabled);
        Logger.recordOutput("Shooter/RightFlywheelEnabled", ShooterConstants.rightFlywheelEnabled);
        Logger.recordOutput("Shooter/LeftFlywheelSetpoint", leftFlywheelSetpoint);
        Logger.recordOutput("Shooter/RightFlywheelSetpoint", rightFlywheelSetpoint);
        Logger.recordOutput("Shooter/LeftSpinSetpoint", leftSpinSetpoint);
        Logger.recordOutput("Shooter/RightSpinSetpoint", rightSpinSetpoint);
        Logger.recordOutput("Shooter/SpinRatio", spinRatio.get());
        Logger.recordOutput("Shooter/LeftFlywheelFailed", leftFlywheelFailed);
        Logger.recordOutput("Shooter/RightFlywheelFailed", rightFlywheelFailed);
        Logger.recordOutput("Shooter/InLimpMode", isInLimpMode());
    }

    /**
     * Update distance-based shot interpolation map from tunable values. Called each periodic cycle to allow real-time
     * tuning.
     */
    private void updateShotMaps() {
        distanceToVelocityMap.clear();

        // Update velocity map
        distanceToVelocityMap.put(dist1m.get(), vel1m.get());
        distanceToVelocityMap.put(dist2m.get(), vel2m.get());
        distanceToVelocityMap.put(dist3m.get(), vel3m.get());
        distanceToVelocityMap.put(dist4m.get(), vel4m.get());
        distanceToVelocityMap.put(dist5m.get(), vel5m.get());
    }

    /**
     * Set velocity for both flywheels independently. This allows for limp-mode operation if one side fails. Respects
     * ShooterConstants enable flags for each motor. Also sets spin motors based on the tunable spin ratio.
     *
     * @param leftRPM Target velocity for left flywheel in RPM
     * @param rightRPM Target velocity for right flywheel in RPM
     */
    public void setFlywheelVelocities(double leftRPM, double rightRPM) {
        leftFlywheelSetpoint = leftRPM;
        rightFlywheelSetpoint = rightRPM;

        // Calculate spin motor velocities based on spin ratio
        double currentSpinRatio = spinRatio.get();
        leftSpinSetpoint = leftRPM * currentSpinRatio;
        rightSpinSetpoint = rightRPM * currentSpinRatio;

        // Only send commands to enabled and non-failed motors
        if (ShooterConstants.leftFlywheelEnabled && !leftFlywheelFailed) {
            io.setLeftFlywheelVelocity(RPM.of(leftRPM));
            io.setLeftSpinVelocity(RPM.of(leftSpinSetpoint));
        } else {
            io.setLeftFlywheelVelocity(RPM.of(0.0)); // Stop disabled/failed motor
            io.setLeftSpinVelocity(RPM.of(0.0));
        }

        if (ShooterConstants.rightFlywheelEnabled && !rightFlywheelFailed) {
            io.setRightFlywheelVelocity(RPM.of(rightRPM));
            io.setRightSpinVelocity(RPM.of(rightSpinSetpoint));
        } else {
            io.setRightFlywheelVelocity(RPM.of(0.0)); // Stop disabled/failed motor
            io.setRightSpinVelocity(RPM.of(0.0));
        }
    }

    /**
     * Set spin motor velocities directly.
     *
     * @param leftRPM Target velocity for left spin motor in RPM
     * @param rightRPM Target velocity for right spin motor in RPM
     */
    public void setSpinVelocities(double leftRPM, double rightRPM) {
        leftSpinSetpoint = leftRPM;
        rightSpinSetpoint = rightRPM;

        if (ShooterConstants.leftFlywheelEnabled && !leftFlywheelFailed) {
            io.setLeftSpinVelocity(RPM.of(leftRPM));
        } else {
            io.setLeftSpinVelocity(RPM.of(0.0));
        }

        if (ShooterConstants.rightFlywheelEnabled && !rightFlywheelFailed) {
            io.setRightSpinVelocity(RPM.of(rightRPM));
        } else {
            io.setRightSpinVelocity(RPM.of(0.0));
        }
    }

    /**
     * Set velocity for both flywheels to the same value. Uses independent control to prevent "fighting" between motors.
     *
     * @param velocityRPM Target velocity in RPM
     */
    public void setFlywheelVelocity(double velocityRPM) {
        setFlywheelVelocities(velocityRPM, velocityRPM);
    }

    /**
     * Prepare shooter for a shot at a specific distance. Automatically calculates and sets the appropriate flywheel
     * velocity.
     *
     * @param distanceMeters Distance to target in meters
     * @return Target velocity in RPM for the given distance
     */
    public double prepareForDistance(double distanceMeters) {
        double targetVelocity = distanceToVelocityMap.get(distanceMeters);
        setFlywheelVelocity(targetVelocity);

        Logger.recordOutput("Shooter/AutoAim/Distance", distanceMeters);
        Logger.recordOutput("Shooter/AutoAim/CalculatedVelocity", targetVelocity);

        return targetVelocity;
    }

    /** Stop all shooter motors. */
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
     * Check if flywheels are at target velocity.
     *
     * @return true if both active flywheels are within tolerance
     */
    @AutoLogOutput(key = "Shooter/AtTargetVelocity")
    public boolean atTargetVelocity() {
        double toleranceRPM = ShooterConstants.flywheelVelocityTolerance.in(RotationsPerSecond) * 60.0;
        boolean leftAtTarget =
                leftFlywheelFailed || Math.abs(inputs.leftFlywheelVelocityRPM - leftFlywheelSetpoint) < toleranceRPM;
        boolean rightAtTarget =
                rightFlywheelFailed || Math.abs(inputs.rightFlywheelVelocityRPM - rightFlywheelSetpoint) < toleranceRPM;

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
     * Detect motor failures based on velocity tracking. A motor is considered failed if there's excessive difference
     * between the two flywheels that cannot be explained by different setpoints.
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

    /** Reset failure flags (for testing or after maintenance). */
    public void resetFailureFlags() {
        leftFlywheelFailed = false;
        rightFlywheelFailed = false;
    }

    // ========== Tunable PID Getters ==========

    /** Get current flywheel kP from NetworkTables. */
    public double getFlywheelKP() {
        return flywheelKP.get();
    }

    /** Get current flywheel kI from NetworkTables. */
    public double getFlywheelKI() {
        return flywheelKI.get();
    }

    /** Get current flywheel kD from NetworkTables. */
    public double getFlywheelKD() {
        return flywheelKD.get();
    }

    /** Get current flywheel kV from NetworkTables. */
    public double getFlywheelKV() {
        return flywheelKV.get();
    }

    /** Get current flywheel kS from NetworkTables. */
    public double getFlywheelKS() {
        return flywheelKS.get();
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
     * Command to stop all shooter motors.
     *
     * @return Command that stops the shooter
     */
    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("StopShooter");
    }

    /**
     * Command to prepare shooter for a shot at a specific distance. Automatically calculates and sets appropriate
     * velocity.
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
     * @return Command that prepares and waits
     */
    public Command spinUpAndWait(double velocityRPM) {
        return spinUpFlywheels(velocityRPM).andThen(waitUntilReady()).withName("SpinUpAndWait");
    }

    /**
     * Command to prepare shooter for a distance and wait until ready.
     *
     * @param distanceMeters Distance to target in meters
     * @return Command that prepares for distance and waits
     */
    public Command prepareForDistanceAndWait(double distanceMeters) {
        return prepareForDistanceCommand(distanceMeters)
                .andThen(waitUntilReady())
                .withName("PrepareForDistanceAndWait");
    }
}
