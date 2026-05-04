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

/**
 * Container class for the left and right shooter subsystems. Handles initialization based on the robot mode and
 * EnabledSubsystems flags.
 */
public class Shooter {
    private final BaseShooter left;
    private final BaseShooter right;

    public Shooter() {
        BaseShooterIO leftIO;
        BaseShooterIO rightIO;

        // Use YAMS implementation if available, fallback to existing logic if needed
        // Choosing BaseShooterYAMS as it handles both REAL and SIM through passive simulation
        leftIO = new BaseShooterYAMS(null, ShooterConstants.leftConfig);
        rightIO = new BaseShooterYAMS(null, ShooterConstants.rightConfig);

        left = new BaseShooter(leftIO, ShooterConstants.leftConfig);
        right = new BaseShooter(rightIO, ShooterConstants.rightConfig);

        // Finalize YAMS binding by associating with finalized subsystems
        ((BaseShooterYAMS)leftIO).setSubsystem(left);
        ((BaseShooterYAMS)rightIO).setSubsystem(right);
    }

    public BaseShooter getLeft() {
        return left;
    }

    public BaseShooter getRight() {
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

    /** Set independent flywheel velocities for each side. */
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
