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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

/**
 * Container class for the left and right shooter subsystems. Handles initialization based on the robot mode and
 * EnabledSubsystems flags.
 */
public class Shooter {
    private final @NotNull BaseShooter left;
    private final @NotNull BaseShooter right;

    public Shooter() {
        BaseShooterIO leftIO;
        BaseShooterIO rightIO;

        // Choose Left IO
        if (Constants.EnabledSubsystems.kShooterLeft) {
            leftIO = switch (Constants.currentMode) {
                case REAL -> new BaseShooterIOKrakenX60(ShooterConstants.leftConfig);
                case SIM -> new BaseShooterIOSim(ShooterConstants.leftConfig);
                default -> new BaseShooterIO() {};};
        } else {
            leftIO = new BaseShooterIO() {};
        }

        // Choose Right IO
        if (Constants.EnabledSubsystems.kShooterRight) {
            rightIO = switch (Constants.currentMode) {
                case REAL -> new BaseShooterIOKrakenX60(ShooterConstants.rightConfig);
                case SIM -> new BaseShooterIOSim(ShooterConstants.rightConfig);
                default -> new BaseShooterIO() {};};
        } else {
            rightIO = new BaseShooterIO() {};
        }

        this.left = new BaseShooter(leftIO, ShooterConstants.leftConfig);
        this.right = new BaseShooter(rightIO, ShooterConstants.rightConfig);
    }

    public @NotNull BaseShooter getLeft() {
        return this.left;
    }

    public @NotNull BaseShooter getRight() {
        return this.right;
    }

    /** Stop both left and right shooters. */
    public void stop() {
        this.left.stop();
        this.right.stop();
    }

    /** Set flywheel velocity for both left and right shooters. */
    public void setFlywheelVelocity(AngularVelocity velocity) {
        this.left.setFlywheelVelocity(velocity);
        this.right.setFlywheelVelocity(velocity);
    }

    /** Set independent flywheel velocities for each side. */
    public void setFlywheelVelocities(AngularVelocity leftVelocity, AngularVelocity rightVelocity) {
        this.left.setFlywheelVelocity(leftVelocity);
        this.right.setFlywheelVelocity(rightVelocity);
    }

    /** Unified stop command for both shooters. */
    public Command stopCommand() {
        return this.left.stopCommand().alongWith(this.right.stopCommand());
    }

    public boolean isRunning() {
        return this.left.isRunning() || this.right.isRunning();
    }
}
