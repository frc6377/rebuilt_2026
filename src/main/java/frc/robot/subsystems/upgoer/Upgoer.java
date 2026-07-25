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
import frc.robot.util.NerfModeController;
import org.littletonrobotics.junction.Logger;

/** Feeder subsystem that pushes game pieces into the shooter. */
public class Upgoer extends SubsystemBase {
    private final UpgoerIO io;
    private final UpgoerIOInputsAutoLogged inputs = new UpgoerIOInputsAutoLogged();
    private final String logName;
    private final double multiplier;
    private final NerfModeController nerfModeController;

    private AngularVelocity setpoint = RPM.of(0.0);

    /**
     * @param io The IO implementation to use.
     * @param logName Logging key prefix (e.g. "LeftShooterUpgoer").
     */
    public Upgoer(UpgoerIO io, String logName, double multiplier, NerfModeController nerfModeController) {
        this.io = io;
        this.logName = logName;
        this.multiplier = multiplier;
        this.nerfModeController = nerfModeController;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(logName, inputs);
        Logger.recordOutput(logName + "/Setpoint", setpoint);
        Logger.recordOutput(logName + "/Running", Math.abs(setpoint.in(RPM)) > 1.0);
        Logger.recordOutput(logName + "/AtTargetVelocity", atTargetVelocity());
        Logger.recordOutput(
                logName + "/CurrentCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    /** Set the feeder velocity. */
    public void setVelocity(AngularVelocity velocity) {
        setpoint = velocity;
        io.setVelocity(velocity.times(multiplier));
    }

    /** Stop the feeder. */
    public void stop() {
        setpoint = RPM.of(0.0);
        io.stop();
    }

    public Command runVelocityCommand(AngularVelocity velocity) {
        return Commands.startEnd(() -> setVelocity(velocity), this::stop, this).withName("UpgoerRunVelocity");
    }

    public Command feedCommand() {
        return runVelocityCommand(nerfModeController.getUpgoerConstants().defaultFeedVelocity())
                .withName("UpgoerFeed");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("UpgoerStop");
    }

    public AngularVelocity getVelocity() {
        return inputs.velocity;
    }

    public boolean atTargetVelocity() {
        double toleranceRpm = 150.0;
        return Math.abs(inputs.velocity.in(RPM) - setpoint.in(RPM)) <= toleranceRpm;
    }
}
