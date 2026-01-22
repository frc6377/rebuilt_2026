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

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,
    }

    public static final class CANIDs {
        public static final int SHOOTER_MOTOR = 7;
    }

    public static final class ShooterConstants {
        public static final InvertedValue shooterOuttakeDirection = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue shooterNeutralMode = NeutralModeValue.Coast;
        public static final MotorOutputConfigs shooterMotorOutputConfigs =
                new MotorOutputConfigs().withInverted(shooterOuttakeDirection).withNeutralMode(shooterNeutralMode);

        // PID gains for velocity control (Slot0)
        // kP: Proportional gain - output per unit of error in velocity
        // kI: Integral gain - output per unit of integrated error
        // kD: Derivative gain - output per unit of error derivative
        // kS: Static feedforward - output to overcome static friction
        // kV: Velocity feedforward - output per unit of requested velocity
        public static final Slot0Configs shooterGains =
                new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.12);
        public static final PIDController shooterSimPIDController = new PIDController(0.1, 0.0, 0.0);
        public static final TalonFXConfiguration shooterTalonFXConfiguration = new TalonFXConfiguration()
                .withMotorOutput(shooterMotorOutputConfigs)
                .withSlot0(shooterGains);

        // Simulation Constants
        public static final double MOI = 0.01;
        public static final double gearRatio = 1.0;
        public static final int motorCount = 1;
    }
}
