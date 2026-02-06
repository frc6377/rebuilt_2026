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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Feeder subsystem that pushes game pieces into the shooter. */
public class Upgoer extends SubsystemBase {
    private final UpgoerIO io;
    private final UpgoerIOInputsAutoLogged inputs = new UpgoerIOInputsAutoLogged();

    private AngularVelocity setpoint = RPM.of(0.0);

    public Upgoer(UpgoerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Upgoer", inputs);
        Logger.recordOutput("Upgoer/Setpoint", setpoint);
    }

    /** Set the feeder velocity. */
    public void setVelocity(AngularVelocity velocity) {
        setpoint = velocity;
        io.setVelocity(velocity);
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
        return runVelocityCommand(UpgoerConstants.defaultFeedVelocity).withName("UpgoerFeed");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("UpgoerStop");
    }

    public AngularVelocity getVelocity() {
        return inputs.velocity;
    }

    @AutoLogOutput(key = "Upgoer/AtTargetVelocity")
    public boolean atTargetVelocity() {
        double toleranceRpm = 150.0;
        return Math.abs(inputs.velocity.in(RPM) - setpoint.in(RPM)) <= toleranceRpm;
    }
}
