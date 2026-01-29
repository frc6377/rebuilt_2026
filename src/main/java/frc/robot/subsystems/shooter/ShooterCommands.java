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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
    /**
     * Command to spin up flywheels to a target velocity.
     *
     * @param shooter The shooter subsystem
     * @param velocityRPM Target velocity in RPM
     * @return Command that runs the flywheels
     */
    public static Command spinUpFlywheels(Shooter shooter, double velocityRPM) {
        return Commands.run(() -> shooter.setFlywheelVelocity(velocityRPM), shooter)
                .withName("SpinUpFlywheels");
    }

    /**
     * Command to spin up flywheels with a dynamic velocity supplier.
     *
     * @param shooter The shooter subsystem
     * @param velocitySupplier Supplier for target velocity in RPM
     * @return Command that runs the flywheels
     */
    public static Command spinUpFlywheels(Shooter shooter, DoubleSupplier velocitySupplier) {
        return Commands.run(() -> shooter.setFlywheelVelocity(velocitySupplier.getAsDouble()), shooter)
                .withName("SpinUpFlywheels");
    }

    /**
     * Command to set hood angle.
     *
     * @param shooter The shooter subsystem
     * @param angleDegrees Target angle in degrees
     * @return Command that sets the hood angle
     */
    public static Command setHoodAngle(Shooter shooter, double angleDegrees) {
        return Commands.runOnce(() -> shooter.setHoodAngle(angleDegrees), shooter).withName("SetHoodAngle");
    }

    /**
     * Command to set hood angle with a dynamic angle supplier.
     *
     * @param shooter The shooter subsystem
     * @param angleSupplier Supplier for target angle in degrees
     * @return Command that sets the hood angle
     */
    public static Command setHoodAngle(Shooter shooter, DoubleSupplier angleSupplier) {
        return Commands.run(() -> shooter.setHoodAngle(angleSupplier.getAsDouble()), shooter)
                .withName("SetHoodAngle");
    }

    /**
     * Command to stop all shooter motors.
     *
     * @param shooter The shooter subsystem
     * @return Command that stops the shooter
     */
    public static Command stopShooter(Shooter shooter) {
        return Commands.runOnce(() -> shooter.stop(), shooter).withName("StopShooter");
    }

    /**
     * Command to prepare shooter for a shot (spin up and set hood angle).
     *
     * @param shooter The shooter subsystem
     * @param velocityRPM Target flywheel velocity in RPM
     * @param angleDegrees Target hood angle in degrees
     * @return Command that prepares the shooter
     */
    public static Command prepareShot(Shooter shooter, double velocityRPM, double angleDegrees) {
        return Commands.parallel(spinUpFlywheels(shooter, velocityRPM), setHoodAngle(shooter, angleDegrees))
                .withName("PrepareShot");
    }

    /**
     * Command to wait until flywheels are at target velocity.
     *
     * @param shooter The shooter subsystem
     * @return Command that waits until ready
     */
    public static Command waitUntilReady(Shooter shooter) {
        return Commands.waitUntil(() -> shooter.atTargetVelocity()).withName("WaitUntilReady");
    }

    /**
     * Command to prepare shooter and wait until ready.
     *
     * @param shooter The shooter subsystem
     * @param velocityRPM Target flywheel velocity in RPM
     * @param angleDegrees Target hood angle in degrees
     * @return Command that prepares and waits
     */
    public static Command prepareShotAndWait(Shooter shooter, double velocityRPM, double angleDegrees) {
        return prepareShot(shooter, velocityRPM, angleDegrees)
                .andThen(waitUntilReady(shooter))
                .withName("PrepareShotAndWait");
    }
}
