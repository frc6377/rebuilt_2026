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

package frc.robot.subsystems.state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Robot state manager for high-level modes and simulation-only counts. */
public class RobotState extends SubsystemBase {
    public enum Mode {
        IDLE,
        SHOOTING,
        SHUTTLING,
        DEFENSE,
        CLIMBING
    }

    public enum Zone {
        UNKNOWN,
        PROTECTED,
        MIDDLE,
        OPPONENT
    }

    private static RobotState instance;

    @AutoLogOutput
    private Mode mode = Mode.IDLE;

    @AutoLogOutput
    private int simGamePieceCount = 0;

    @AutoLogOutput
    private Zone fieldZone = Zone.PROTECTED;

    private Supplier<Pose2d> poseSupplier = Pose2d::new;

    private final LoggedNetworkNumber simMaxGamePieces = new LoggedNetworkNumber("RobotState/Sim/MaxGamePieces", 5.0);
    private final LoggedNetworkNumber protectedZoneFraction =
            new LoggedNetworkNumber("RobotState/Zone/ProtectedFraction", 0.33);
    private final LoggedNetworkNumber middleZoneFraction =
            new LoggedNetworkNumber("RobotState/Zone/MiddleFraction", 0.33);

    private RobotState() {}

    public static RobotState create() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    public static RobotState getInstance() {
        return instance;
    }

    public void setMode(Mode newMode) {
        mode = newMode;
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isClimbing() {
        return mode == Mode.CLIMBING;
    }

    public boolean isShooting() {
        return mode == Mode.SHOOTING;
    }

    public boolean isShuttling() {
        return mode == Mode.SHUTTLING;
    }

    public boolean isDefense() {
        return mode == Mode.DEFENSE;
    }

    public int getSimGamePieceCount() {
        return simGamePieceCount;
    }

    public void setSimGamePieceCount(int count) {
        simGamePieceCount = clamp(count);
    }

    public void incrementSimGamePieceCount() {
        simGamePieceCount = clamp(simGamePieceCount + 1);
    }

    public void decrementSimGamePieceCount() {
        simGamePieceCount = clamp(simGamePieceCount - 1);
    }

    private int clamp(int value) {
        int max = (int) Math.round(simMaxGamePieces.get());
        if (value < 0) {
            return 0;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

    public Command setModeCommand(Mode newMode) {
        return Commands.runOnce(() -> setMode(newMode), this).withName("SetRobotMode:" + newMode);
    }

    public Command incrementSimGamePieces() {
        return Commands.runOnce(this::incrementSimGamePieceCount, this).withName("SimGamePieces+1");
    }

    public Command decrementSimGamePieces() {
        return Commands.runOnce(this::decrementSimGamePieceCount, this).withName("SimGamePieces-1");
    }

    public Command setSimGamePieces(int count) {
        return Commands.runOnce(() -> setSimGamePieceCount(count), this).withName("SetSimGamePieces:" + count);
    }

    public void setPoseSupplier(Supplier<Pose2d> supplier) {
        poseSupplier = supplier != null ? supplier : Pose2d::new;
    }

    @Override
    public void periodic() {
        updateZoneMode();
    }

    private void updateZoneMode() {
        if (mode == Mode.CLIMBING) {
            return; // Do not override climb mode
        }

        Pose2d pose = poseSupplier.get();
        double fieldLengthMeters = FieldConstants.FIELD_LENGTH.in(edu.wpi.first.units.Units.Meters);
        double xMeters = pose.getTranslation().getX();

        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        double distanceFromOwnWall = isRed ? fieldLengthMeters - xMeters : xMeters;

        double protectedFraction = Math.max(0.0, Math.min(1.0, protectedZoneFraction.get()));
        double middleFraction = Math.max(0.0, Math.min(1.0, middleZoneFraction.get()));
        double protectedMeters = fieldLengthMeters * protectedFraction;
        double middleMeters = fieldLengthMeters * middleFraction;

        if (distanceFromOwnWall <= protectedMeters) {
            mode = Mode.SHOOTING;
            fieldZone = Zone.PROTECTED;
        } else if (distanceFromOwnWall <= protectedMeters + middleMeters) {
            mode = Mode.SHUTTLING;
            fieldZone = Zone.MIDDLE;
        } else {
            mode = Mode.DEFENSE;
            fieldZone = Zone.OPPONENT;
        }
    }
}
