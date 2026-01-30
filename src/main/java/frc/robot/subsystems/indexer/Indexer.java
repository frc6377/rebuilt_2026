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

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final IndexerTuning tuning = new IndexerTuning();

    // Track last applied PID values to avoid unnecessary updates
    private Slot0Configs lastAppliedConfig = null;

    /** Creates a new Indexer. */
    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer speed (RPS)", inputs.IndexerVelocity.in(RotationsPerSecond));

        // Check for updated PID values and apply if changed
        Slot0Configs currentConfig;
        if (Constants.currentMode == Constants.Mode.SIM) {
            currentConfig = tuning.getSimSlot0Configs();
        } else {
            currentConfig = tuning.getSlot0Configs();
        }

        if (hasConfigChanged(currentConfig)) {
            io.updatePIDConfig(currentConfig);
            lastAppliedConfig = currentConfig;
        }
    }

    /**
     * Checks if the PID/FF configuration has changed from the last applied values.
     *
     * @param currentConfig The current configuration from tuning.
     * @return true if the configuration has changed.
     */
    private boolean hasConfigChanged(Slot0Configs currentConfig) {
        if (lastAppliedConfig == null) {
            return true;
        }
        return currentConfig.kP != lastAppliedConfig.kP
                || currentConfig.kI != lastAppliedConfig.kI
                || currentConfig.kD != lastAppliedConfig.kD
                || currentConfig.kS != lastAppliedConfig.kS
                || currentConfig.kV != lastAppliedConfig.kV;
    }

    // /** Run the Indexer at the specified voltage. */
    // public void setVoltage(Voltage volts) {
    //     io.setVoltage(volts);
    // }

    /** Run the Indexer at the specified velocity. */
    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    /**
     * Shoot based on distance using the tuning map.
     *
     * @param distance The distance to the target.
     */
    public void shootAtDistance(Distance distance) {
        AngularVelocity targetVelocity = tuning.getAngularVelocity(distance);
        io.setVelocity(targetVelocity);
    }

    /** Stop the Indexer. */
    public void stop() {
        io.stop();
    }

    /** Returns the current velocity in rad/s. */
    public AngularVelocity getVelocity() {
        return inputs.IndexerVelocity;
    }
    /**
     * Returns true if the Indexer is at the target velocity within tolerance.
     *
     * @param targetVelocity The target velocity to check against.
     * @return true if at target velocity.
     */
    public boolean atTargetVelocity(AngularVelocity targetVelocity) {
        double currentVel = inputs.IndexerVelocity.in(RadiansPerSecond);
        double targetVel = targetVelocity.in(RadiansPerSecond);
        return Math.abs(currentVel - targetVel) < IndexerConstants.kVelocityTolerance.in(RadiansPerSecond);
    }

    // ==================== Command Factory Methods ====================

    /**
     * Creates a command to run the Indexer at a specified velocity. The command runs until interrupted.
     *
     * @param velocity The target velocity.
     * @return A command that runs the Indexer at the specified velocity.
     */
    public Command runVelocityCommand(AngularVelocity velocity) {
        return runEnd(() -> setVelocity(velocity), () -> setVelocity(RotationsPerSecond.zero()))
                .withName("Indexer.RunVelocity");
    }

    /**
     * Creates a command to run the Indexer at a velocity supplied dynamically. The command runs until interrupted.
     *
     * @param velocitySupplier A supplier for the target velocity.
     * @return A command that runs the Indexer at the supplied velocity.
     */
    public Command runVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
        return Commands.run(() -> setVelocity(velocitySupplier.get()), this).withName("Indexer.RunVelocityDynamic");
    }

    /**
     * Creates a command to spin up the Indexer to a target velocity and finish when at speed. Stops the Indexer when
     * the command ends.
     *
     * @param velocity The target velocity.
     * @return A command that spins up and finishes when at velocity.
     */
    public Command spinUpCommand(AngularVelocity velocity) {
        return Commands.run(() -> setVelocity(velocity), this)
                .until(() -> atTargetVelocity(velocity))
                .withName("Indexer.SpinUp");
    }

    /**
     * Creates a command to spin up the Indexer based on distance. The command runs until interrupted.
     *
     * @param distance The distance to the target.
     * @return A command that runs the Indexer at the velocity for the given distance.
     */
    public Command shootAtDistanceCommand(Distance distance) {
        return Commands.run(() -> shootAtDistance(distance), this).withName("Indexer.ShootAtDistance");
    }

    /**
     * Creates a command to spin up the Indexer based on a dynamically supplied distance. The command runs until
     * interrupted.
     *
     * @param distanceSupplier A supplier for the distance to the target.
     * @return A command that runs the Indexer at the velocity for the supplied distance.
     */
    public Command shootAtDistanceCommand(Supplier<Distance> distanceSupplier) {
        return Commands.run(() -> shootAtDistance(distanceSupplier.get()), this)
                .withName("Indexer.ShootAtDistanceDynamic");
    }

    /**
     * Creates a command to stop the Indexer.
     *
     * @return A command that stops the Indexer.
     */
    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("Indexer.Stop");
    }

    /**
     * Creates a command to idle the Indexer (keep running at a low speed for quick spin-up).
     *
     * @return A command that idles the Indexer.
     */
    public Command idleCommand() {
        return Commands.run(() -> setVelocity(IndexerConstants.kIdleVelocity), this)
                .withName("Indexer.Idle");
    }
}
