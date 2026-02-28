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
import frc.robot.subsystems.shooter.left.*;
import frc.robot.subsystems.shooter.right.*;

/**
 * Container class for the left and right shooter subsystems. Handles initialization based on the robot mode and
 * EnabledSubsystems flags.
 */
public class Shooter {
    private final LeftShooter left;
    private final RightShooter right;

    public Shooter() {
        LeftShooterIO leftIO;
        RightShooterIO rightIO;

        // Choose Left IO
        if (Constants.EnabledSubsystems.kShooterLeft) {
            switch (Constants.currentMode) {
                case REAL:
                    leftIO = new LeftShooterIOKrakenX60();
                    break;
                case SIM:
                    leftIO = new LeftShooterIOSim();
                    break;
                default:
                    leftIO = new LeftShooterIO() {};
                    break;
            }
        } else {
            leftIO = new LeftShooterIO() {};
        }

        // Choose Right IO
        if (Constants.EnabledSubsystems.kShooterRight) {
            switch (Constants.currentMode) {
                case REAL:
                    rightIO = new RightShooterIOKrakenX60();
                    break;
                case SIM:
                    rightIO = new RightShooterIOSim();
                    break;
                default:
                    rightIO = new RightShooterIO() {};
                    break;
            }
        } else {
            rightIO = new RightShooterIO() {};
        }

        left = new LeftShooter(leftIO);
        right = new RightShooter(rightIO);
    }

    public LeftShooter getLeft() {
        return left;
    }

    public RightShooter getRight() {
        return right;
    }

    /** Stop both left and right shooters. */
    public void stop() {
        left.stop();
        right.stop();
    }

    /** Set flywheel velocity for both left and right shooters. */
    public void setFlywheelVelocity(AngularVelocity velocity) {
        left.setFlywheelVelocity(velocity);
        right.setFlywheelVelocity(velocity);
    }

    public void setFlywheelVelocities(AngularVelocity leftVelocity, AngularVelocity rightVelocity) {
        left.setFlywheelVelocity(leftVelocity);
        right.setFlywheelVelocity(rightVelocity);
    }
    /** Unified stop command for both shooters. */
    public Command stopCommand() {
        return left.stopCommand().alongWith(right.stopCommand());
    }

    public boolean isRunning() {
        return left.isRunning() || right.isRunning();
    }
}
