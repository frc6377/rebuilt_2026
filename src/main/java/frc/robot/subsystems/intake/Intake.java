package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.extender.Extender;
import frc.robot.subsystems.intake.extender.ExtenderIO;
import frc.robot.subsystems.intake.roller.Roller;
import frc.robot.subsystems.intake.roller.RollerIO;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final Extender extender;
    private final Roller roller;

    public Intake(ExtenderIO extenderIO, RollerIO rollerIO) {
        this.extender = new Extender(extenderIO);
        this.roller = new Roller(rollerIO);
    }

    public Extender getExtender() {
        return extender;
    }

    public Roller getRoller() {
        return roller;
    }

    public void setNeutralMode(NeutralModeValue mode) {
        var extenderMotor = extender.getMotor();
        if (extenderMotor != null)
            extenderMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
        var rollerMotor = roller.getMotor();
        if (rollerMotor != null) rollerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                "Intake/CurrentCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    public Command extendIntake() {
        return extender.extendCommand();
    }

    public Command extendIntakeAndWait() {
        return extender.extendAndWaitCommand();
    }

    public Command retractIntakeAndWait() {
        return extender.retractAndWaitCommand();
    }

    public Command stowIntake() {
        return extender.retractCommand();
    }

    public Command toggleIntake() {
        return extender.toggleCommand();
    }

    public Command goToSiftAngleOneCommand() {
        return extender.goToSiftAngleOneCommand();
    }

    public Command goToSiftAngleTwoCommand() {
        return extender.goToSiftAngleTwoCommand();
    }

    public Command goToCustomAngleOneCommand() {
        return extender.goToCustomAngleOneCommand();
    }

    public Command goToCustomAngleTwoCommand() {
        return extender.goToCustomAngleTwoCommand();
    }

    public Command zeroIntake() {
        return extender.zeroCommand();
    }

    public Command stop() {
        return extender.stopCommand();
    }

    public Command siftFuelCommand() {
        return extender.siftFuelSpeedCommand();
    }

    public Command intakeRollerCommand() {
        return roller.runCommand();
    }

    public Command outtakeRollerCommand() {
        return roller.outtakeCommand();
    }

    public Command stopRollerCommand() {
        return roller.stopCommand();
    }

    public boolean isRollerRunning() {
        return roller.isRunning();
    }

    public BooleanSupplier isRollerRunningSupplier() {
        return this::isRollerRunning;
    }

    public Command intakeCommand() {
        return extender.extendAndWaitCommand().andThen(roller.runCommand());
    }

    public Command retractIntakeCommand() {
        return roller.stopCommand().andThen(extender.retractAndWaitCommand());
    }

    public Command outtakeCommand() {
        return extender.extendCommand().andThen(roller.outtakeCommand());
    }
}
