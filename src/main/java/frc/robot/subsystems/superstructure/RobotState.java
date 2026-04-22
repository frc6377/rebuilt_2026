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

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import java.util.function.Supplier;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
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
    private @NotNull Zone fieldZone = Zone.PROTECTED;

    private @NotNull Supplier<Pose2d> poseSupplier = Pose2d::new;

    private final LoggedNetworkNumber simMaxGamePieces = new LoggedNetworkNumber("RobotState/Sim/MaxGamePieces", 5.0);
    private final LoggedNetworkNumber protectedZoneFraction =
            new LoggedNetworkNumber("RobotState/Zone/ProtectedFraction", 0.33);
    private final LoggedNetworkNumber middleZoneFraction =
            new LoggedNetworkNumber("RobotState/Zone/MiddleFraction", 0.33);

    private RobotState() {}

    public static @NotNull RobotState create() {
        if (null == instance) {
            instance = new RobotState();
        }
        return instance;
    }

    public static RobotState getInstance() {
        return instance;
    }

    public void setMode(Mode newMode) {
        this.mode = newMode;
    }

    public Mode getMode() {
        return this.mode;
    }

    public Zone getFieldZone() {
        return this.fieldZone;
    }

    public boolean isClimbing() {
        return Mode.CLIMBING == mode;
    }

    public boolean isShooting() {
        return Mode.SHOOTING == mode;
    }

    public boolean isShuttling() {
        return Mode.SHUTTLING == mode;
    }

    public boolean isDefense() {
        return Mode.DEFENSE == mode;
    }

    public int getSimGamePieceCount() {
        return this.simGamePieceCount;
    }

    public void setSimGamePieceCount(int count) {
        this.simGamePieceCount = this.clamp(count);
    }

    public void incrementSimGamePieceCount() {
        this.simGamePieceCount = this.clamp(this.simGamePieceCount + 1);
    }

    public void decrementSimGamePieceCount() {
        this.simGamePieceCount = this.clamp(this.simGamePieceCount - 1);
    }

    private int clamp(int value) {
        int max = (int) Math.round(this.simMaxGamePieces.get());
        if (0 > value) {
            return 0;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

    public Command setModeCommand(Mode newMode) {
        return Commands.runOnce(() -> this.setMode(newMode), this).withName("SetRobotMode:" + newMode);
    }

    public Command incrementSimGamePieces() {
        return Commands.runOnce(this::incrementSimGamePieceCount, this).withName("SimGamePieces+1");
    }

    public Command decrementSimGamePieces() {
        return Commands.runOnce(this::decrementSimGamePieceCount, this).withName("SimGamePieces-1");
    }

    public Command setSimGamePieces(int count) {
        return Commands.runOnce(() -> this.setSimGamePieceCount(count), this).withName("SetSimGamePieces:" + count);
    }

    public void setPoseSupplier(@Nullable Supplier<Pose2d> supplier) {
        this.poseSupplier = null != supplier ? supplier : Pose2d::new;
    }

    @Override
    public void periodic() {
        this.updateZoneMode();
    }

    private void updateZoneMode() {
        if (Mode.CLIMBING == mode) {
            return; // Do not override climb mode
        }

        Pose2d pose = this.poseSupplier.get();
        double fieldLengthMeters = FieldConstants.fieldLength;
        double xMeters = pose.getTranslation().getX();

        boolean isRed = Alliance.Red == DriverStation.getAlliance().orElse(Alliance.Blue);
        double distanceFromOwnWall = isRed ? fieldLengthMeters - xMeters : xMeters;

        double protectedFraction = Math.max(0.0, Math.min(1.0, this.protectedZoneFraction.get()));
        double middleFraction = Math.max(0.0, Math.min(1.0, this.middleZoneFraction.get()));
        double protectedMeters = fieldLengthMeters * protectedFraction;
        double middleMeters = fieldLengthMeters * middleFraction;

        if (distanceFromOwnWall <= protectedMeters) {
            this.mode = Mode.SHOOTING;
            this.fieldZone = Zone.PROTECTED;
        } else if (distanceFromOwnWall <= protectedMeters + middleMeters) {
            this.mode = Mode.SHUTTLING;
            this.fieldZone = Zone.MIDDLE;
        } else {
            this.mode = Mode.DEFENSE;
            this.fieldZone = Zone.OPPONENT;
        }
    }
}
