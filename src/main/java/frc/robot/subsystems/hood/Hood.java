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

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Hood subsystem for adjustable pitch control. Uses Kraken x44 motor for position-controlled adjustable hood. */
public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    // Tunable PID constants
    private final LoggedNetworkNumber kP = new LoggedNetworkNumber("Hood/KP", HoodConstants.defaultKP);
    private final LoggedNetworkNumber kI = new LoggedNetworkNumber("Hood/KI", HoodConstants.defaultKI);
    private final LoggedNetworkNumber kD = new LoggedNetworkNumber("Hood/KD", HoodConstants.defaultKD);

    // Setpoint
    private double angleSetpoint = 0.0;

    public Hood(HoodIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Skip if hood is disabled
        if (!ShooterConstants.hoodEnabled) {
            Logger.recordOutput("Hood/Enabled", false);
            return;
        }

        // Update and log inputs
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        // Log setpoint
        Logger.recordOutput("Hood/Enabled", true);
        Logger.recordOutput("Hood/AngleSetpoint", angleSetpoint);
    }

    /**
     * Set hood angle in degrees.
     *
     * @param angleDegrees Target angle in degrees
     */
    public void setAngle(double angleDegrees) {
        if (!ShooterConstants.hoodEnabled) return;
        angleSetpoint = angleDegrees;
        io.setAngle(Degrees.of(angleDegrees));
    }

    /** Stop hood motor. */
    public void stop() {
        if (!ShooterConstants.hoodEnabled) return;
        angleSetpoint = 0.0;
        io.stop();
    }

    /**
     * Get current hood angle.
     *
     * @return Angle in degrees (0.0 if hood disabled)
     */
    public double getAngle() {
        if (!ShooterConstants.hoodEnabled) return 0.0;
        return inputs.angleDegrees;
    }

    /**
     * Check if hood is enabled.
     *
     * @return true if hood functionality is enabled
     */
    public boolean isEnabled() {
        return ShooterConstants.hoodEnabled;
    }

    /**
     * Get PID gain kP.
     *
     * @return kP value
     */
    public double getKP() {
        return kP.get();
    }

    /**
     * Get PID gain kI.
     *
     * @return kI value
     */
    public double getKI() {
        return kI.get();
    }

    /**
     * Get PID gain kD.
     *
     * @return kD value
     */
    public double getKD() {
        return kD.get();
    }

    // ========== Command Factory Methods ==========

    /**
     * Command to set hood angle.
     *
     * @param angleDegrees Target angle in degrees
     * @return Command that sets the hood angle
     */
    public Command setAngleCommand(double angleDegrees) {
        return Commands.runOnce(() -> setAngle(angleDegrees), this).withName("SetHoodAngle");
    }

    /**
     * Command to set hood angle with a dynamic angle supplier.
     *
     * @param angleSupplier Supplier for target angle in degrees
     * @return Command that sets the hood angle
     */
    public Command setAngleCommand(DoubleSupplier angleSupplier) {
        return Commands.run(() -> setAngle(angleSupplier.getAsDouble()), this).withName("SetHoodAngle");
    }

    /**
     * Command to stop hood motor.
     *
     * @return Command that stops the hood
     */
    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("StopHood");
    }
}
