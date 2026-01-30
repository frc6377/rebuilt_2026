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

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public boolean IndexerMotorConnected = true;
        public Angle IndexerPosition = Radians.of(0.0);
        public AngularVelocity IndexerVelocity = RadiansPerSecond.of(0.0);
        public Voltage IndexerAppliedVolts = Volts.of(0);
        public Current IndexerCurrent = Amps.of(0);
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(IndexerIOInputs inputs) {}

    /** Run the Indexer motor at the specified voltage. */
    default void setVoltage(Voltage volts) {}

    /** Run the Indexer motor at the specified velocity. */
    default void setVelocity(AngularVelocity velocity) {}

    /** Stop the Indexer motor. */
    default void stop() {}

    /** Update PID/FF configuration with new values. */
    default void updatePIDConfig(Slot0Configs config) {}
}
