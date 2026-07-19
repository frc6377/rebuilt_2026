// Copyright 2021-2025 FRC 6328
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
import edu.wpi.first.math.util.Units;
import frc.robot.util.TalonFXCurrentConfigurator;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and CANcoder.
 * Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFXReal extends ModuleIOTalonFX {
    // Queue to read inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;
    private final TalonFXCurrentConfigurator driveCurrentConfigurator;

    public ModuleIOTalonFXReal(SwerveModuleConstants constants) {
        super(constants);

        driveCurrentConfigurator = new TalonFXCurrentConfigurator(
                "Drive-" + driveTalon.getDeviceID(), driveTalon.getConfigurator(), driveCurrentLimitsConfig);
        this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.drivePosition);
        this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.turnAbsolutePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);
        logCurrentConfigurator(
                "Drive/CurrentLimit/Motor" + driveTalon.getDeviceID(), driveCurrentConfigurator.snapshot());

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble(pos -> Units.rotationsToRadians(pos) / constants.DriveMotorGearRatio)
                .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveSupplyCurrentLimit(double currentLimitAmps) {
        driveCurrentConfigurator.requestSupplyCurrentLimit(currentLimitAmps);
    }

    private static void logCurrentConfigurator(String key, TalonFXCurrentConfigurator.Snapshot snapshot) {
        Logger.recordOutput(key + "/RequestedLimitAmps", snapshot.requestedLimitAmps());
        Logger.recordOutput(key + "/LastSuccessfulAcknowledgedLimitAmps", snapshot.lastSuccessfulLimitAmps());
        Logger.recordOutput(key + "/RequestedRevision", snapshot.requestedRevision());
        Logger.recordOutput(key + "/LastSuccessfulAcknowledgedRevision", snapshot.lastSuccessfulRevision());
        Logger.recordOutput(key + "/LastOutcome", snapshot.lastOutcome().name());
        Logger.recordOutput(key + "/LastStatusName", snapshot.lastStatusName());
        Logger.recordOutput(key + "/LastStatusDescription", snapshot.lastStatusDescription());
        Logger.recordOutput(key + "/LastException", snapshot.lastException());
        Logger.recordOutput(key + "/LastAttemptAgeSeconds", snapshot.lastAttemptAgeSeconds());
        Logger.recordOutput(
                key + "/LastSuccessfulAcknowledgedApplyAgeSeconds", snapshot.lastSuccessfulApplyAgeSeconds());
        Logger.recordOutput(key + "/AttemptCount", snapshot.attemptCount());
        Logger.recordOutput(key + "/SuccessCount", snapshot.successCount());
        Logger.recordOutput(key + "/FailureCount", snapshot.failureCount());
        Logger.recordOutput(key + "/ExceptionCount", snapshot.exceptionCount());
        Logger.recordOutput(key + "/RetryAttemptCount", snapshot.retryAttemptCount());
        Logger.recordOutput(key + "/DeduplicatedRequestCount", snapshot.deduplicatedRequestCount());
        Logger.recordOutput(key + "/Pending", snapshot.pending());
        Logger.recordOutput(key + "/Retrying", snapshot.retrying());
        Logger.recordOutput(key + "/InFlight", snapshot.inFlight());
        Logger.recordOutput(key + "/Closed", snapshot.closed());
        Logger.recordOutput(key + "/WorkerAlive", snapshot.workerAlive());
    }
}
