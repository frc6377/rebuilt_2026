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

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public Angle angle = Degrees.of(0.0);
        public Voltage appliedVoltage = Volts.of(0.0);
        public Current current = Amps.of(0.0);
        public Temperature temp = Celsius.of(0.0);
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(HoodIOInputs inputs) {}

    /** Set hood angle */
    default void setAngle(Angle angle) {}

    /** Stop hood motor */
    default void stop() {}
}
