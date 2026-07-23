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
import frc.robot.util.NerfModeController;

/**
 * Container class for the left and right shooter subsystems. Handles initialization based on the robot mode and
 * EnabledSubsystems flags.
 */
public class Shooter {
    private final BaseShooter left;
    private final BaseShooter right;
    private final NerfModeController nerfModeController;

    public Shooter(NerfModeController nerfModeController) {
        this.nerfModeController = nerfModeController;
        BaseShooterIO leftIO;
        BaseShooterIO rightIO;

        ShooterConstants constants = nerfModeController.getShooterConstants();

        // Choose Left IO
        if (Constants.EnabledSubsystems.kShooterLeft) {
            leftIO = switch (Constants.currentMode) {
                case REAL -> new BaseShooterIOKrakenX60(constants.leftConfig());
                case SIM -> new BaseShooterIOSim(constants.leftConfig(), constants);
                default -> new BaseShooterIO() {};};
        } else {
            leftIO = new BaseShooterIO() {};
        }

        // Choose Right IO
        if (Constants.EnabledSubsystems.kShooterRight) {
            rightIO = switch (Constants.currentMode) {
                case REAL -> new BaseShooterIOKrakenX60(constants.rightConfig());
                case SIM -> new BaseShooterIOSim(constants.rightConfig(), constants);
                default -> new BaseShooterIO() {};};
        } else {
            rightIO = new BaseShooterIO() {};
        }

        left = new BaseShooter(leftIO, constants.leftConfig(), constants);
        right = new BaseShooter(rightIO, constants.rightConfig(), constants);
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
