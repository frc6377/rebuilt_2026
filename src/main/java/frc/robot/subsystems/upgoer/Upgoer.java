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

package frc.robot.subsystems.upgoer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

/** Feeder subsystem that pushes game pieces into the shooter. */
public class Upgoer extends SubsystemBase {
    private final UpgoerIO io;
    private final UpgoerIOInputsAutoLogged inputs = new UpgoerIOInputsAutoLogged();
    private final String logName;
    private final double multiplier;

    private AngularVelocity setpoint = RPM.of(0.0);

    /**
     * @param io The IO implementation to use.
     * @param logName Logging key prefix (e.g. "LeftShooterUpgoer").
     */
    public Upgoer(UpgoerIO io, String logName, double multiplier) {
        this.io = io;
        this.logName = logName;
        this.multiplier = multiplier;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs(this.logName, this.inputs);
        Logger.recordOutput(this.logName + "/Setpoint", this.setpoint);
        Logger.recordOutput(this.logName + "/Running", 1.0 < Math.abs(setpoint.in(RPM)));
        Logger.recordOutput(this.logName + "/AtTargetVelocity", this.atTargetVelocity());
        Logger.recordOutput(
                this.logName + "/CurrentCommand",
                null != getCurrentCommand() ? this.getCurrentCommand().getName() : "None");
    }

    /** Set the feeder velocity. */
    public void setVelocity(@NotNull AngularVelocity velocity) {
        this.setpoint = velocity;
        this.io.setVelocity(velocity.times(this.multiplier));
    }

    /** Stop the feeder. */
    public void stop() {
        this.setpoint = RPM.of(0.0);
        this.io.stop();
    }

    public Command runVelocityCommand(@NotNull AngularVelocity velocity) {
        return Commands.startEnd(() -> this.setVelocity(velocity), this::stop, this).withName("UpgoerRunVelocity");
    }

    public Command feedCommand() {
        return this.runVelocityCommand(UpgoerConstants.defaultFeedVelocity).withName("UpgoerFeed");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("UpgoerStop");
    }

    public AngularVelocity getVelocity() {
        return this.inputs.velocity;
    }

    public boolean atTargetVelocity() {
        double toleranceRpm = 150.0;
        return Math.abs(this.inputs.velocity.in(RPM) - this.setpoint.in(RPM)) <= toleranceRpm;
    }
}
