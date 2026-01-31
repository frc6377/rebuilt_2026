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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public boolean indexerConnected = true;
        public Angle indexerPosition = Radians.of(0.0);
        public AngularVelocity indexerVelocity = RadiansPerSecond.of(0.0);
        public Voltage indexerAppliedVolts = Volts.of(0);
        public Current indexerCurrent = Amps.of(0);
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(IndexerIOInputs inputs) {}

    /** Run the indexer at the specified duty cycle (-1.0 to 1.0). */
    default void setSpeed(double speed) {}

    /** Stop the indexer. */
    default void stop() {}
}
