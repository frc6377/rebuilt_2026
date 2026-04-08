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
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

    @AutoLog
    class RollerIOInputs {
        public double leaderSpeedPercentile = 0.0;
        public Voltage leaderAppliedVolts = Volts.zero();
        public AngularVelocity leaderVelocity = RotationsPerSecond.zero();
        public Current leaderStatorCurrent = Amps.zero();
        public Temperature leaderMotorTemp = Celsius.zero();
        public double leaderClosedLoopOutput = 0.0;

        public double followerSpeedPercentile = 0.0;
        public Voltage followerAppliedVolts = Volts.zero();
        public AngularVelocity followerVelocity = RotationsPerSecond.zero();
        public Current followerStatorCurrent = Amps.zero();
        public Temperature followerMotorTemp = Celsius.zero();
        public double followerClosedLoopOutput = 0.0;

        public boolean isRunning = false;
        public String currentControl = "None";
        public AngularVelocity setpoint = RPM.zero();
    }

    default void updateInputs(RollerIOInputs inputs) {}

    default void setMode(NeutralModeValue mode) {}

    default void setMotorPercentage(double percent) {}

    default void start() {}

    default void stop() {}

    default void outtake() {}

    default void setRollerVoltage(Voltage volts) {}

    default int getIntakedFuel() {
        return 0;
    }

    default boolean isRunning() {
        return false;
    }

    default void periodic() {}

    default void idle() {}
}
