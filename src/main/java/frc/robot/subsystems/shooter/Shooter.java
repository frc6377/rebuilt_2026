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

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    /** Creates a new Shooter. */
    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    /** Run the shooter at the specified voltage. */
    public void setVoltage(Voltage volts) {
        io.setVoltage(volts);
    }

    /** Run the shooter at the specified velocity. */
    public void setVelocity(AngularVelocity velocity) {
        io.setVelocity(velocity);
    }

    /**
     * Shoot based on distance using the tuning map.
     *
     * @param distanceMeters The distance to the target in meters.
     */
    public void shootAtDistance(double distanceMeters) {
        AngularVelocity targetVelocity = ShooterTuning.getAngularVelocity(distanceMeters);
        io.setVelocity(targetVelocity);
    }

    /** Stop the shooter. */
    public void stop() {
        io.stop();
    }

    /** Returns the current velocity in rad/s. */
    public AngularVelocity getVelocity() {
        return inputs.shooterVelocity;
    }
}
