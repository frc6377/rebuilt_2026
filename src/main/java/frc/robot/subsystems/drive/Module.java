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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final SwerveModuleConstants constants;
    private final @NotNull String logKey;

    private final @NotNull Alert driveDisconnectedAlert;
    private final @NotNull Alert turnDisconnectedAlert;
    private final @NotNull Alert turnEncoderDisconnectedAlert;
    private SwerveModulePosition @NotNull [] odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, int index, SwerveModuleConstants constants) {
        this.io = io;
        this.constants = constants;
        this.logKey = "Drive/Module" + index;
        this.driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + index + ".", AlertType.kError);
        this.turnDisconnectedAlert = new Alert("Disconnected turn motor on module " + index + ".", AlertType.kError);
        this.turnEncoderDisconnectedAlert =
                new Alert("Disconnected turn encoder on module " + index + ".", AlertType.kError);
    }

    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs(this.logKey, this.inputs);

        // Calculate positions for odometry
        int sampleCount = this.inputs.odometryTimestamps.length; // All signals are sampled together
        this.odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = this.inputs.odometryDrivePositionsRad[i] * this.constants.WheelRadius;
            Rotation2d angle = this.inputs.odometryTurnPositions[i];
            this.odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        this.driveDisconnectedAlert.set(!this.inputs.driveConnected);
        this.turnDisconnectedAlert.set(!this.inputs.turnConnected);
        this.turnEncoderDisconnectedAlert.set(!this.inputs.turnEncoderConnected);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(@NotNull SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(this.getAngle());
        state.cosineScale(this.inputs.turnAbsolutePosition);

        // Apply setpoints
        this.io.setDriveVelocity(state.speedMetersPerSecond / this.constants.WheelRadius);
        this.io.setTurnPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        this.io.setDriveOpenLoop(output);
        this.io.setTurnPosition(new Rotation2d());
    }

    public void runCharacterizationTurning(double output) {
        this.io.setTurnOpenLoop(0);
        this.io.setTurnOpenLoop(output);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        this.io.setDriveOpenLoop(0.0);
        this.io.setTurnOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return this.inputs.turnAbsolutePosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return this.inputs.drivePositionRad * this.constants.WheelRadius;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return this.inputs.driveVelocityRadPerSec * this.constants.WheelRadius;
    }

    /** Returns the module position (turn angle and drive position). */
    public @NotNull SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.getPositionMeters(), this.getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public @NotNull SwerveModuleState getState() {
        return new SwerveModuleState(this.getVelocityMetersPerSec(), this.getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return this.odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return this.inputs.odometryTimestamps;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return this.inputs.drivePositionRad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(this.inputs.driveVelocityRadPerSec);
    }
}
