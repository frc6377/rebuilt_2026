package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static IntakeIO intake;
    private static IntakeIO.IntakeIOInputs inputs;

    public Intake(IntakeIO intakeIO) {
        intake = intakeIO;
        inputs = new IntakeIO.IntakeIOInputs();
    }

    public void intake() {
        intake.start();
    }

    public void outtake() {
        intake.outtake();
    }

    public void stopIntake() {
        intake.stop();
    }

    public Command intakeCommand() {
        return run(() -> intake());
    }

    public Command outtakeCommand() {
        return run(() -> outtake());
    }

    public Command stopIntakeCommand() {
        return run(() -> stopIntake());
    }

    @Override
    public void periodic() {
        intake.updateInputs(inputs);
    }
}
