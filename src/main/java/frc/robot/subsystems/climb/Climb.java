// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimberIO.ClimberIOInputs;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

    private final ClimberIO climberIO;
    private final ClimberIOInputs inputs;

    private Angle currentPivotSetpoint = ClimbConstants.kPivotStowAngle;

    public Climb(ClimberIO climberIO) {
        this.climberIO = (ClimbConstants.kDisabled) ? new ClimberIO() {} : climberIO;
        this.inputs = new ClimberIO.ClimberIOInputs();
    }

    public boolean atPivotSetpoint() {
        return inputs.pivotAngle.isNear(currentPivotSetpoint, ClimbConstants.kPivotAngleTolerance);
    }


    public boolean isHookOnPole() {
        return inputs.hookStatorCurrent.gte(ClimbConstants.kHookContactCurrentThreshold);
    }


    public boolean isClimbDone() {
        return inputs.hookStatorCurrent.gte(ClimbConstants.kClimbDoneCurrentThreshold);
    }

    public Command goToPivotAngle(Angle angle) {
        return runOnce(() -> {
            climberIO.goToPivotAngle(angle);
            currentPivotSetpoint = angle;
        });
    }

    public Command extendArm() {
        return goToPivotAngle(Degrees.of(ClimbConstants.kPivotMaxAngleRad));
    }

    public Command stowArm() {
        return goToPivotAngle(Degrees.of(ClimbConstants.kPivotMinAngleRad));
    }

    public Command manualPivot(double percent) {
        return runEnd(
                () -> climberIO.set(percent),
                () -> climberIO.set(0));
    }

    public Command rotateHook() {
        return runEnd(
                () -> climberIO.setHookPercent(ClimbConstants.kHookRotateSpeed),
                () -> climberIO.setHookPercent(0))
                .until(this::isHookOnPole);
    }

 
    public Command climb() {
        return runEnd(
                () -> climberIO.setHookPercent(ClimbConstants.kHookClimbSpeed),
                () -> climberIO.setHookPercent(0))
                .until(this::isClimbDone);
    }

    public Command manualHook(double percent) {
        return runEnd(
                () -> climberIO.setHookPercent(percent),
                () -> climberIO.setHookPercent(0));
    }

    // -------------------------------------------------------------------------
    // Combined commands
    // -------------------------------------------------------------------------

    /**
     * Full autonomous climb sequence:
     *   1. Extend pivot arm to pole
     *   2. Wait until pivot is at setpoint
     *   3. Spin hook until pole contact (low current spike)
     *   4. Continue spinning hook to lift robot (high current spike)
     *   5. Hold — both motors stay in brake mode
     */
    public Command climbSequence() {
        return Commands.sequence(
                extendArm(),
                Commands.waitUntil(this::atPivotSetpoint),
                rotateHook(),
                climb(),
                holdPosition());
    }

    /**
     * Lock both motors in brake mode.
     * Runs indefinitely — interrupt with stop() or stowArm().
     */
    public Command holdPosition() {
        return runOnce(() -> climberIO.stop());
    }

    /**
     * Emergency abort — stops all motors and retracts the pivot.
     * Interrupts any running command on this subsystem.
     */
    public Command abort() {
        return Commands.sequence(
                runOnce(() -> climberIO.stop()),
                stowArm());
    }

    // -------------------------------------------------------------------------
    // Encoder management
    // -------------------------------------------------------------------------

    /**
     * Re-seed both TalonFX encoders from the through bore encoders.
     * Call this on robot enable via a default command or RobotContainer.
     */
    public Command resetToAbsolute() {
        return runOnce(() -> climberIO.resetToAbsolute());
    }

    public Command zeroEncoders() {
        return runOnce(() -> climberIO.zeroEncoder());
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(inputs);
        climberIO.periodic();

        Logger.recordOutput("Climb/PivotAngle (Rotations)",    inputs.pivotAngle);
        Logger.recordOutput("Climb/HookAngle (Rotations)",     inputs.hookAngle);
        Logger.recordOutput("Climb/HookStatorCurrent (A)",     inputs.hookStatorCurrent);
        Logger.recordOutput("Climb/PivotStatorCurrent (A)",    inputs.pivotStatorCurrent);
        Logger.recordOutput("Climb/AtPivotSetpoint",           atPivotSetpoint());
        Logger.recordOutput("Climb/IsHookOnPole",              isHookOnPole());
        Logger.recordOutput("Climb/IsClimbDone",               isClimbDone());
        Logger.recordOutput("Climb/PivotSetpoint (Rotations)", currentPivotSetpoint);
    }
}