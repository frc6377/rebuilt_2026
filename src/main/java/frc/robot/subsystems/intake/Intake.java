package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.extender.ExtenderIO;
import frc.robot.subsystems.intake.extender.ExtenderIOInputsAutoLogged;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOInputsAutoLogged;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final RollerIO roller;
    private final ExtenderIO extender;
    private final RollerIOInputsAutoLogged rollerInputs;
    private final ExtenderIOInputsAutoLogged extenderInputs;

    public Intake(RollerIO rollerIO, ExtenderIO extenderIO) {
        roller = rollerIO;
        extender = extenderIO;
        rollerInputs = new RollerIOInputsAutoLogged();
        extenderInputs = new ExtenderIOInputsAutoLogged();
    }

    public int getIntakedFuel() {
        return roller.getIntakedFuel();
    }

    // Extender Commands
    public Command extendIntake() {
        return runOnce(extender::extend);
    }

    public Command extendIntakeAndWait() {
        return runOnce(extender::extend).until(extender.isExtended());
    }

    public Command stowIntake() {
        return runOnce(extender::retract);
    }

    public Command toggleIntake() {
        // return new ConditionalCommand(
        // runOnce(() -> extender.retract()), runOnce(() -> extender.extend()),
        // extender.isExtended());
        return runOnce(extender::toggle);
    }

    // Roller Commands
    public Command intakeRollerCommand() {
        return Commands.runEnd(roller::start, roller::stop, this);
    }

    public Command outtakeRollerCommand() {
        return Commands.runEnd(roller::outtake, roller::stop, this);
    }

    public Command stopRollerCommand() {
        return runOnce(roller::stop);
    }

    // Combination Commands
    public Command intakeCommand() {
        return currentRunDescend().until(extender.isExtended()).andThen(runEnd(roller::start, roller::stop));
    }

    public Command retractIntakeCommand() {
        return runOnce(roller::stop).andThen(currentRunShoot());
    }

    public Command goToSiftAngleOneCommand() {
        return runOnce(extender::goToSiftAngleOne);
    }

    public Command goToSiftAngleTwoCommand() {
        return runOnce(extender::goToSiftAngleTwo);
    }

    public Command outtakeCommand() {
        return runOnce(extender::extend).andThen(runEnd(roller::outtake, roller::stop));
    }

    public Command siftFuelCommand() {
        return Commands.repeatingSequence(
                run(extender::goToSiftAngleOne).until(extender.atTarget()),
                run(extender::goToSiftAngleTwo).until(extender.atTarget()));
    }

    public Command zeroExtender() {
        return runOnce(extender::zero);
    }

    public boolean isRollerRunning() {
        return Math.abs(rollerInputs.rollerSpeedPercentile) > 0.1;
    }

    public BooleanSupplier isRollerRunningSupplier() {
        return this::isRollerRunning;
    }

    public Command zeroIntake() {
        return runOnce(extender::zero);
    }

    public Command currentRunShoot() {
        return run(() -> extender.currentRunShoot(-2))
                .withTimeout(1)
                .andThen(() -> extender.currentRunShoot(-2))
                .until(() -> extender.getCurrent().gte(Amps.of(20)))
                .andThen(stop());
    }

    public Command currentRunDescend() {
        return run(() -> extender.currentRunShoot(1))
                .until(() -> extender.getCurrent().gte(Amps.of(15)))
                .andThen(stop());
    }

    public Command autoZeroIntakeCommand() {
        return Commands.runEnd(extender::autoZero, extender::stop, this).until(extender.atTarget());
    }

    public Command stop() {
        return runOnce(extender::stop);
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
