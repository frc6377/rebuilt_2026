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

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

//
/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    /** Field dimensions and hub positions for 2026 FUEL game */
    public static final class FieldConstants {
        // Standard FRC field dimensions
        public static final Distance FIELD_LENGTH = Feet.of(54.0); // ~16.46m
        public static final Distance FIELD_WIDTH = Feet.of(27.0); // ~8.23m

        // Hub dimensions
        public static final Distance HUB_OPENING_HEIGHT = Inches.of(72.0); // 1.83m
        public static final Distance HUB_OPENING_DIAMETER = Inches.of(41.7); // ~1.06m hexagonal

        // Hub is centered 158.6in (4.03m) from alliance wall, centered on field width
        public static final Distance HUB_DISTANCE_FROM_WALL =
                Inches.of(158.6).plus(HUB_OPENING_DIAMETER.div(2)); // 4.03m

        // Blue alliance hub position (blue wall is at x = 0)
        public static final Translation2d BLUE_HUB_POSITION =
                new Translation2d(HUB_DISTANCE_FROM_WALL.in(Meters), FIELD_WIDTH.in(Meters) / 2.0);

        // Red alliance hub position (red wall is at x = field length)
        public static final Translation2d RED_HUB_POSITION = new Translation2d(
                FIELD_LENGTH.in(Meters) - HUB_DISTANCE_FROM_WALL.in(Meters), FIELD_WIDTH.in(Meters) / 2.0);

        /** Get the hub position for the current alliance */
        public static Translation2d getHubPosition() {
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                    ? RED_HUB_POSITION
                    : BLUE_HUB_POSITION;
        }
    }

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,
    }
}
