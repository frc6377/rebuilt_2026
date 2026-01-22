package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;

public class IntakeRollerSubsystem extends SubsystemBase {
    private static IntakeIO intakeMotor;

    public IntakeRollerSubsystem(IntakeIO intakeIO) {
        intakeMotor = intakeIO;
    }

    public static void intakeCommand() {
        intakeMotor.setSpeed(RollerConstants.INTAKE_SPEED);
    }

    public static void outtakeCommand() {
        intakeMotor.setSpeed(RollerConstants.OUTAKE_SPEED);
    }

    @Override
    public void periodic() {
        intakeMotor.periodic();
    }
}
