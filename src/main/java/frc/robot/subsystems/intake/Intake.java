package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.subsystems.intake.extender.ExtenderIO;
import frc.robot.subsystems.intake.roller.RollerIO;

public class Intake extends SubsystemBase {
    private static RollerIO roller;
    private static ExtenderIO extender;
    private static RollerIO.IntakeIOInputs rollerInputs;
    private static ExtenderIO.ExtenderIOInputs extenderInputs;

    public Intake(RollerIO rollerIO, ExtenderIO extenderIO) {
        roller = rollerIO;
        extender = extenderIO;
        rollerInputs = new RollerIO.IntakeIOInputs();
    }

    public Command setPosition(Angle position) {
        return runOnce(() -> extender.setPosition(position));
    }

    // Extender Commands
    public Command extendIntake() {
        return runOnce(() -> extender.extend());
    }

    public Command stowIntake() {
        return runOnce(() -> extender.retract());
    }

    // Roller Commands
    public Command intakeRollerCommand() {
        return run(() -> roller.start());
    }

    public Command outtakeRollerCommand() {
        return run(() -> roller.outtake());
    }

    public Command stopRollerCommand() {
        return run(() -> roller.stop());
    }

    public Command intakeCommand() {
        return runOnce(() -> extendIntake()).andThen(runEnd(() -> intakeRollerCommand(), () -> stopRollerCommand()));
    }

    public Command retractIntakeCommand() {
        return runOnce(() -> stopRollerCommand()).andThen(runOnce(() -> stowIntake()));
    }

    public Command siftFuelCommand() {
        return Commands.runEnd(
                () -> Commands.repeatingSequence(
                        setPosition(ExtenderConstants.kExtenderSiftAngleOne),
                        setPosition(ExtenderConstants.kExtenderSiftAngleTwo)),
                () -> extendIntake(),
                this);
    }

    @Override
    public void periodic() {
        roller.updateInputs(rollerInputs);
        extender.updateInputs(extenderInputs);
        extender.periodic();
    }
}
