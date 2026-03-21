package frc.robot.subsystems.intake;

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
        extender.setMode(mode);
        roller.setMode(mode);
    }

    public void setExtenderPidEnabled(boolean enabled) {
        extender.setPidEnabled(enabled);
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
        // return new ConditionalCommand(
        // runOnce(() -> extender.retract()), runOnce(() -> extender.extend()),
        // extender.isExtended());
        return runOnce(extender::toggle);
    }

    public BooleanSupplier isRetracted() {
        return extender.isRetracted();
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
        return extender.siftFuelPositionCommand();
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

    public Command currentRunShootManual() {
        return run(() -> extender.currentRunShoot(-1));
    }

    public Command currentRunDescend() {
        return run(() -> extender.currentRunShoot(1))
                .until(() -> extender.getCurrent().gte(Amps.of(8)))
                .andThen(stop());
    }

    public Command setNeutralModeBrake() {
        return runOnce(() -> extender.setNeutralMode(NeutralModeValue.Brake));
    }

    public Command setNeutralModeCoast() {
        return runOnce(() -> extender.setNeutralMode(NeutralModeValue.Coast));
    }

    public Command voltageSiftFuel() {

        return run(() -> extender.currentRunShoot(0))
                .until(() -> extender.getCurrent().gte(Amps.of(8)))
                .andThen(Commands.repeatingSequence(
                        run(() -> extender.currentRunShoot(0))
                                .until(() -> extender.getCurrent().gte(Amps.of(8)))
                                .withTimeout(2),
                        run(() -> extender.currentRunShoot(0))
                                .until(() -> extender.getCurrent().gte(Amps.of(8)))
                                .withTimeout(2)));
    }

    public Command currentRunDescendNoCheck() {
        return run(() -> extender.currentRunShoot(1));
    }

    public Command autoZeroIntakeCommand() {
        return Commands.runEnd(extender::autoZero, extender::stop, this).until(extender.atTarget());
    }

    @Override
    public void periodic() {
        roller.updateInputs(rollerInputs);
        extender.updateInputs(extenderInputs);
        extender.periodic();
        roller.periodic();

        // Use processInputs to log IO structs efficiently (avoids per-cycle
        // allocation/retention from recordOutput
        // Unit types)
        Logger.processInputs("Intake/Extender", extenderInputs);
        Logger.processInputs("Intake/Roller", rollerInputs);

        Logger.recordOutput(
                "Intake/CurrentCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }
}
