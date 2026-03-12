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

package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

    @AutoLog
    class RollerIOInputs {
        public double rollerSpeedPercentile = 0.0;
        public Voltage rollerAppliedVolts = Volts.of(0.0);
        public AngularVelocity rollerVelocity = RotationsPerSecond.of(0.0);
        public Current statorCurrent = Amps.zero();
        public Temperature motorTemp = Celsius.of(0.0);
    }

    default void updateInputs(RollerIOInputs inputs) {}

    default void start() {}

    default void stop() {}

    default void outtake() {}

    default int getIntakedFuel() {
        return 0;
    }

    default void periodic() {}
}
