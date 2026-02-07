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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
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
    private AngularVelocity leftFlywheelSetpoint = RPM.of(0.0);
    private AngularVelocity rightFlywheelSetpoint = RPM.of(0.0);
    private AngularVelocity leftSpinSetpoint = RPM.of(0.0);
    private AngularVelocity rightSpinSetpoint = RPM.of(0.0);

    // Limp mode state
    private boolean leftFlywheelFailed = false;
    private boolean rightFlywheelFailed = false;
    private int failureDetectionCounter = 0;
    private static final int FAILURE_DETECTION_CYCLES = 25; // ~0.5s at 50Hz
    private int accelMismatchCounter = 0;
    private double lastLeftVelocityRPM = 0.0;
    private double lastRightVelocityRPM = 0.0;
    private boolean lastSetpointsActive = false;

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
     * @param leftVelocity Target velocity for left flywheel
     * @param rightVelocity Target velocity for right flywheel
     */
    public void setFlywheelVelocities(AngularVelocity leftVelocity, AngularVelocity rightVelocity) {
        leftFlywheelSetpoint = leftVelocity;
        rightFlywheelSetpoint = rightVelocity;

        // Calculate spin motor velocities based on spin ratio
        double currentSpinRatio = spinRatio.get();
        leftSpinSetpoint = RPM.of(leftVelocity.in(RPM) * currentSpinRatio);
        rightSpinSetpoint = RPM.of(rightVelocity.in(RPM) * currentSpinRatio);

        // Only send commands to enabled and non-failed motors
        if (ShooterConstants.leftFlywheelEnabled && !leftFlywheelFailed) {
            io.setLeftFlywheelVelocity(leftVelocity);
            io.setLeftSpinVelocity(leftSpinSetpoint);
        } else {
            io.setLeftFlywheelVelocity(RPM.of(0.0)); // Stop disabled/failed motor
            io.setLeftSpinVelocity(RPM.of(0.0));
        }

        if (ShooterConstants.rightFlywheelEnabled && !rightFlywheelFailed) {
            io.setRightFlywheelVelocity(rightVelocity);
            io.setRightSpinVelocity(rightSpinSetpoint);
        } else {
            io.setRightFlywheelVelocity(RPM.of(0.0)); // Stop disabled/failed motor
            io.setRightSpinVelocity(RPM.of(0.0));
        }
    }

    /**
     * Set spin motor velocities directly.
     *
     * @param leftVelocity Target velocity for left spin motor
     * @param rightVelocity Target velocity for right spin motor
     */
    public void setSpinVelocities(AngularVelocity leftVelocity, AngularVelocity rightVelocity) {
        leftSpinSetpoint = leftVelocity;
        rightSpinSetpoint = rightVelocity;

        if (ShooterConstants.leftFlywheelEnabled && !leftFlywheelFailed) {
            io.setLeftSpinVelocity(leftVelocity);
        } else {
            io.setLeftSpinVelocity(RPM.of(0.0));
        }

        if (ShooterConstants.rightFlywheelEnabled && !rightFlywheelFailed) {
            io.setRightSpinVelocity(rightVelocity);
        } else {
            io.setRightSpinVelocity(RPM.of(0.0));
        }
    }

    /**
     * Set velocity for both flywheels to the same value. Uses independent control to prevent "fighting" between motors.
     *
     * @param velocity Target velocity
     */
    public void setFlywheelVelocity(AngularVelocity velocity) {
        setFlywheelVelocities(velocity, velocity);
    }

    /**
     * Prepare shooter for a shot at a specific distance. Automatically calculates and sets the appropriate flywheel
     * velocity.
     *
     * @param distance Distance to target
     * @return Target velocity for the given distance
     */
    public AngularVelocity prepareForDistance(Distance distance) {
        double distanceMeters = distance.in(Meters);
        double targetVelocityRPM = distanceToVelocityMap.get(distanceMeters);
        AngularVelocity targetVelocity = RPM.of(targetVelocityRPM);
        setFlywheelVelocity(targetVelocity);

        Logger.recordOutput("Shooter/AutoAim/Distance", distanceMeters);
        Logger.recordOutput("Shooter/AutoAim/CalculatedVelocity", targetVelocityRPM);

        return targetVelocity;
    }

    /** Stop all shooter motors. */
    public void stop() {
        leftFlywheelSetpoint = RPM.of(0.0);
        rightFlywheelSetpoint = RPM.of(0.0);
        io.stop();
    }

    /**
     * Get current left flywheel velocity.
     *
     * @return Velocity
     */
    public AngularVelocity getLeftFlywheelVelocity() {
        return inputs.leftFlywheelVelocity;
    }

    /**
     * Get current right flywheel velocity.
     *
     * @return Velocity
     */
    public AngularVelocity getRightFlywheelVelocity() {
        return inputs.rightFlywheelVelocity;
    }

    /**
     * Check if flywheels are at target velocity.
     *
     * @return true if both active flywheels are within tolerance
     */
    @AutoLogOutput(key = "Shooter/AtTargetVelocity")
    public boolean atTargetVelocity() {
        AngularVelocity tolerance = ShooterConstants.flywheelVelocityTolerance;
        boolean leftAtTarget = leftFlywheelFailed
                || Math.abs(inputs.leftFlywheelVelocity.in(RPM) - leftFlywheelSetpoint.in(RPM)) < tolerance.in(RPM);
        boolean rightAtTarget = rightFlywheelFailed
                || Math.abs(inputs.rightFlywheelVelocity.in(RPM) - rightFlywheelSetpoint.in(RPM)) < tolerance.in(RPM);

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
        if (!ShooterConstants.flywheelSafetyEnabled) {
            failureDetectionCounter = 0;
            accelMismatchCounter = 0;
            lastSetpointsActive = false;
            return;
        }

        // Only check for failures when motors are supposed to be running
        boolean leftEnabled = ShooterConstants.leftFlywheelEnabled;
        boolean rightEnabled = ShooterConstants.rightFlywheelEnabled;
        boolean setpointsActive = leftFlywheelSetpoint.in(RPM) > 100 || rightFlywheelSetpoint.in(RPM) > 100;

        if (setpointsActive && !lastSetpointsActive) {
            // Reset ramp tracking when we start spinning up from zero
            lastLeftVelocityRPM = inputs.leftFlywheelVelocity.in(RPM);
            lastRightVelocityRPM = inputs.rightFlywheelVelocity.in(RPM);
            accelMismatchCounter = 0;
            failureDetectionCounter = 0;
        }

        if (leftEnabled && rightEnabled && setpointsActive) {
            // Compare acceleration rate while ramping up
            double currentLeftRPM = inputs.leftFlywheelVelocity.in(RPM);
            double currentRightRPM = inputs.rightFlywheelVelocity.in(RPM);
            double leftAccel = currentLeftRPM - lastLeftVelocityRPM;
            double rightAccel = currentRightRPM - lastRightVelocityRPM;
            double accelDifference = Math.abs(leftAccel - rightAccel);

            if (leftAccel > 0.0 && rightAccel > 0.0 && accelDifference > ShooterConstants.accelMismatchToleranceRPM) {
                accelMismatchCounter++;
            } else {
                accelMismatchCounter = 0;
            }

            if (accelMismatchCounter >= ShooterConstants.accelMismatchCycles) {
                if (leftAccel < rightAccel) {
                    leftFlywheelFailed = true;
                    Logger.recordOutput("Shooter/Alert", "Left flywheel accel mismatch - entering limp mode");
                } else {
                    rightFlywheelFailed = true;
                    Logger.recordOutput("Shooter/Alert", "Right flywheel accel mismatch - entering limp mode");
                }
            }

            // Calculate expected velocity difference based on setpoints
            double expectedDifference = Math.abs(leftFlywheelSetpoint.in(RPM) - rightFlywheelSetpoint.in(RPM));

            // Calculate actual velocity difference
            double actualDifference =
                    Math.abs(inputs.leftFlywheelVelocity.in(RPM) - inputs.rightFlywheelVelocity.in(RPM));

            // Calculate unexplained difference (potential failure)
            double unexplainedDifference = Math.abs(actualDifference - expectedDifference);

            double maxDifferenceRPM = ShooterConstants.maxVelocityDifference.in(RPM);

            // Only trigger failure if the difference can't be explained by different setpoints
            if (unexplainedDifference > maxDifferenceRPM) {
                failureDetectionCounter++;
            } else {
                failureDetectionCounter = 0;
            }

            if (failureDetectionCounter >= FAILURE_DETECTION_CYCLES) {
                // Determine which motor is likely failed (the slower one relative to its setpoint)
                double leftError = Math.abs(inputs.leftFlywheelVelocity.in(RPM) - leftFlywheelSetpoint.in(RPM));
                double rightError = Math.abs(inputs.rightFlywheelVelocity.in(RPM) - rightFlywheelSetpoint.in(RPM));

                if (leftError > rightError) {
                    leftFlywheelFailed = true;
                    Logger.recordOutput("Shooter/Alert", "Left flywheel failure detected - entering limp mode");
                } else {
                    rightFlywheelFailed = true;
                    Logger.recordOutput("Shooter/Alert", "Right flywheel failure detected - entering limp mode");
                }
            }
        } else {
            failureDetectionCounter = 0;
            accelMismatchCounter = 0;
        }

        lastLeftVelocityRPM = inputs.leftFlywheelVelocity.in(RPM);
        lastRightVelocityRPM = inputs.rightFlywheelVelocity.in(RPM);
        lastSetpointsActive = setpointsActive;
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
     * @param velocity Target velocity
     * @return Command that runs the flywheels
     */
    public Command spinUpFlywheels(AngularVelocity velocity) {
        return Commands.run(() -> setFlywheelVelocity(velocity), this).withName("SpinUpFlywheels");
    }

    /**
     * Command to spin up flywheels with a dynamic velocity supplier.
     *
     * @param velocitySupplier Supplier for target velocity
     * @return Command that runs the flywheels
     */
    public Command spinUpFlywheels(Supplier<AngularVelocity> velocitySupplier) {
        return Commands.run(() -> setFlywheelVelocity(velocitySupplier.get()), this)
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
     * @param distance Distance to target
     * @return Command that prepares the shooter for the distance
     */
    public Command prepareForDistanceCommand(Distance distance) {
        return Commands.runOnce(() -> prepareForDistance(distance), this).withName("PrepareForDistance");
    }

    /**
     * Command to prepare shooter for a shot at a dynamic distance.
     *
     * @param distanceSupplier Supplier for distance to target
     * @return Command that prepares the shooter for the distance
     */
    public Command prepareForDistanceCommand(Supplier<Distance> distanceSupplier) {
        return Commands.run(() -> prepareForDistance(distanceSupplier.get()), this)
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
     * @param velocity Target flywheel velocity
     * @return Command that prepares and waits
     */
    public Command spinUpAndWait(AngularVelocity velocity) {
        return spinUpFlywheels(velocity).andThen(waitUntilReady()).withName("SpinUpAndWait");
    }

    /**
     * Command to prepare shooter for a distance and wait until ready.
     *
     * @param distance Distance to target
     * @return Command that prepares for distance and waits
     */
    public Command prepareForDistanceAndWait(Distance distance) {
        return prepareForDistanceCommand(distance).andThen(waitUntilReady()).withName("PrepareForDistanceAndWait");
    }
}
