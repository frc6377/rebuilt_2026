package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.extender.Extender;
import frc.robot.subsystems.intake.extender.ExtenderIO;
import frc.robot.subsystems.shooter.BaseShooter;
import frc.robot.subsystems.shooter.BaseShooterIO;
import java.util.function.BooleanSupplier;

import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake {
    private final @NotNull Extender extender;
    private final @NotNull BaseShooter roller;

    private final LoggedNetworkNumber tunableIntakeSpeed =
            new LoggedNetworkNumber("Intake/Roller/IntakeSpeed", IntakeConstants.RollerConstants.kIntakeSpeed.in(RPM));
    private final LoggedNetworkNumber tunableOuttakeSpeed = new LoggedNetworkNumber(
            "Intake/Roller/OuttakeSpeed", IntakeConstants.RollerConstants.kOuttakeSpeed.in(RPM));
    private final LoggedNetworkNumber tunableIdleSpeed =
            new LoggedNetworkNumber("Intake/Roller/IdleSpeed", IntakeConstants.RollerConstants.kIdleSpeed.in(RPM));

    public Intake(ExtenderIO extenderIO, @NotNull BaseShooterIO rollerIO) {
        this.extender = new Extender(extenderIO);
        this.roller = new BaseShooter(rollerIO, IntakeConstants.RollerConstants.rollerConfig);

        //        this.roller.setDefaultCommand(this.roller
        //                .spinUpFlywheels(() ->
        //                        IntakeConstants.RollerConstants.kIdleEnabled ? RPM.of(tunableIdleSpeed.get()) :
        // RPM.of(0.0))
        //                .withName("RollerIdle"));

    }

    public @NotNull Extender getExtender() {
        return this.extender;
    }

    public @NotNull BaseShooter getRoller() {
        return this.roller;
    }

    public void setNeutralMode(NeutralModeValue mode) {
        this.extender.setMode(mode);
    }

    public void setExtenderPidEnabled(boolean enabled) {
        this.extender.setPidEnabled(enabled);
    }

    public Command extendIntake() {
        return this.extender.extendCommand();
    }

    public Command extendIntakeAndWait() {
        return this.extender.extendAndWaitCommand();
    }

    public Command retractIntakeAndWait() {
        return this.extender.retractAndWaitCommand();
    }

    public Command stowIntake() {
        return this.extender.retractCommand();
    }

    public Command toggleIntake() {
        return this.extender.toggleCommand();
    }

    public Command goToSiftAngleOneCommand() {
        return this.extender.goToSiftAngleOneCommand();
    }

    public Command goToSiftAngleTwoCommand() {
        return this.extender.goToSiftAngleTwoCommand();
    }

    public Command goToCustomAngleOneCommand() {
        return this.extender.goToCustomAngleOneCommand();
    }

    public Command goToCustomAngleTwoCommand() {
        return this.extender.goToCustomAngleTwoCommand();
    }

    public Command zeroIntake() {
        return this.extender.zeroCommand();
    }

    public Command stop() {
        return this.extender.stopCommand();
    }

    public Command siftFuelCommand() {
        return Commands.repeatingSequence(
                this.extender.toggleSiftCommand(), Commands.waitSeconds(0.75).until(this.extender.isAtTarget()));
    }

    public Command intakeRollerCommand() {
        return this.roller.spinUpFlywheels(() -> RPM.of(this.tunableIntakeSpeed.get())).finallyDo(() -> this.roller.stop());
    }

    public Command outtakeRollerCommand() {
        return this.roller.spinUpFlywheels(() -> RPM.of(this.tunableOuttakeSpeed.get()));
    }

    public Command idleRollerCommand() {
        return this.roller.spinUpFlywheels(() -> RPM.of(this.tunableIdleSpeed.get()));
    }

    public Command stopRollerCommand() {
        return this.roller.stopCommand();
    }

    public boolean isRollerRunning() {
        return this.roller.isRunning();
    }

    public @NotNull BooleanSupplier isRollerRunningSupplier() {
        return this::isRollerRunning;
    }

    public Command intakeCommand() {
        return this.extender.extendCommand().andThen(this.intakeRollerCommand());
    }

    public Command retractIntakeCommand() {
        return this.roller.stopCommand().andThen(this.extender.retractCommand());
    }

    public Command outtakeCommand() {
        return this.extender.extendCommand().andThen(this.outtakeRollerCommand());
    }
}
