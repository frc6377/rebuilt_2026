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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Hooded shooter subsystem with dual-sided IO layer.
 * Features independent Kraken X60 motors for redundancy and limp-mode operation.
 */
public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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
    public boolean atTargetVelocity() {
        boolean leftAtTarget = leftFlywheelFailed
                || Math.abs(inputs.leftFlywheelVelocityRPM - leftFlywheelSetpoint)
                        < ShooterConstants.flywheelVelocityTolerance;
        boolean rightAtTarget = rightFlywheelFailed
                || Math.abs(inputs.rightFlywheelVelocityRPM - rightFlywheelSetpoint)
                        < ShooterConstants.flywheelVelocityTolerance;

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

            if (velocityDifference > ShooterConstants.maxVelocityDifference) {
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
}
