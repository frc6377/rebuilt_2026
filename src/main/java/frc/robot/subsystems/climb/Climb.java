// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private ClimberIO climberIO;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    /** Creates a new Climb. */
    public Climb(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }

    public Command climbUp() {
        return startEnd(
                () -> {
                    climberIO.set(ClimbConstants.kClimbSpeed);
                },
                () -> {
                    climberIO.set(0);
                });
    }

    public Command climbDown() {
        return startEnd(
                () -> {
                    climberIO.set(-ClimbConstants.kClimbSpeed);
                },
                () -> {
                    climberIO.set(0);
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
        Logger.processInputs("Climber", inputs);
    }
}
