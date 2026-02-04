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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Left flywheel (independent control)
        public double leftFlywheelVelocityRPM = 0.0;
        public double leftFlywheelAppliedVolts = 0.0;
        public double leftFlywheelCurrentAmps = 0.0;
        public double leftFlywheelTempCelsius = 0.0;

        // Right flywheel (independent control)
        public double rightFlywheelVelocityRPM = 0.0;
        public double rightFlywheelAppliedVolts = 0.0;
        public double rightFlywheelCurrentAmps = 0.0;
        public double rightFlywheelTempCelsius = 0.0;

        // Left spin motor (Kraken X44 on top of left hood)
        public double leftSpinVelocityRPM = 0.0;
        public double leftSpinAppliedVolts = 0.0;
        public double leftSpinCurrentAmps = 0.0;
        public double leftSpinTempCelsius = 0.0;

        // Right spin motor (Kraken X44 on top of right hood)
        public double rightSpinVelocityRPM = 0.0;
        public double rightSpinAppliedVolts = 0.0;
        public double rightSpinCurrentAmps = 0.0;
        public double rightSpinTempCelsius = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ShooterIOInputs inputs) {}

    /** Set left flywheel velocity */
    default void setLeftFlywheelVelocity(AngularVelocity velocity) {}

    /** Set right flywheel velocity */
    default void setRightFlywheelVelocity(AngularVelocity velocity) {}

    /** Set left spin motor velocity */
    default void setLeftSpinVelocity(AngularVelocity velocity) {}

    /** Set right spin motor velocity */
    default void setRightSpinVelocity(AngularVelocity velocity) {}

    /** Stop flywheel motors */
    default void stop() {}
}
