// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimberIO.ClimberIOInputs;

// Limit Switch, Through Bore Encoder

public class Climb extends SubsystemBase {
    private ClimberIO climberIO;
    private ClimberIOInputs inputs;
    private Distance currentSetpoint = ClimbConstants.kStartHeight;
    /** Creates a new Climb. */
    public Climb(ClimberIO climberIO) {
        this.climberIO = climberIO;
        this.inputs = new ClimberIO.ClimberIOInputs();
    }

    public Command setHeight(Distance height) {
        return runOnce(() -> {
            climberIO.goToHeight(height);
            currentSetpoint = height;
        });
    }

    public Command goToStowed() {
        return setHeight(ClimbConstants.kClimbMinHeight);
    }

    public Command goToExtended() {
        return setHeight(ClimbConstants.kClimbMaxHeight);
    }

    public Command goToMidHeight() {
        return setHeight(ClimbConstants.kClimbMaxHeight.divide(2.0));
    }

    public boolean atSetpoint() {
        return inputs.height.isNear(currentSetpoint, ClimbConstants.kSetpointTolerance);
    }

    public Command climbUp() {
        return startEnd(
                () -> {
                    climberIO.set(ClimbConstants.kClimbSpeed);
                },
                () -> {
                    climberIO.set(ClimbConstants.kClimbSpeed * 0);
                });
    }

    public Command climbDown() {
        return startEnd(
                () -> {
                    climberIO.set(-ClimbConstants.kClimbSpeed);
                },
                () -> {
                    climberIO.set(ClimbConstants.kClimbSpeed * 0);
                });
    }

    public Command extendUp() {
        return Commands.none();
    }

    public Command extendDown() {
        return Commands.none();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        climberIO.updateInputs(inputs);
        climberIO.periodic();
    }
}
