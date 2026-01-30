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

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final ShooterTuning tuning = new ShooterTuning();

    // Track last applied PID values to avoid unnecessary updates
    private Slot0Configs lastAppliedConfig = null;

    /** Creates a new Shooter. */
    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

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

    // /** Run the shooter at the specified voltage. */
    // public void setVoltage(Voltage volts) {
    //     io.setVoltage(volts);
    // }

    /** Run the shooter at the specified velocity. */
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

    /** Stop the shooter. */
    public void stop() {
        io.stop();
    }

    /** Returns the current velocity in rad/s. */
    public AngularVelocity getVelocity() {
        return inputs.shooterVelocity;
    }
}
