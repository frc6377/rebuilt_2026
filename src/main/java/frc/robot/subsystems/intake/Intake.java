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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake {
    private final Extender extender;
    private final BaseShooter roller;
    private boolean activelyIntaking = false;
    /** When true, retract/stow commands no-op so fuel cannot be trapped by a retract. */
    private BooleanSupplier retractBlocked = () -> false;

    private final LoggedNetworkNumber tunableIntakeSpeed =
            new LoggedNetworkNumber("Intake/Roller/IntakeSpeed", IntakeConstants.RollerConstants.kIntakeSpeed.in(RPM));
    private final LoggedNetworkNumber tunableOuttakeSpeed = new LoggedNetworkNumber(
            "Intake/Roller/OuttakeSpeed", IntakeConstants.RollerConstants.kOuttakeSpeed.in(RPM));
    private final LoggedNetworkNumber tunableIdleSpeed =
            new LoggedNetworkNumber("Intake/Roller/IdleSpeed", IntakeConstants.RollerConstants.kIdleSpeed.in(RPM));

    public Intake(ExtenderIO extenderIO, BaseShooterIO rollerIO) {
        this.extender = new Extender(extenderIO);
        this.roller = new BaseShooter(rollerIO, IntakeConstants.RollerConstants.rollerConfig);

        // this.roller.setDefaultCommand(this.roller
        // .spinUpFlywheels(() ->
        // IntakeConstants.RollerConstants.kIdleEnabled ? RPM.of(tunableIdleSpeed.get())
        // :
        // RPM.of(0.0))
        // .withName("RollerIdle"));

    }

    public Extender getExtender() {
        return extender;
    }

    public BaseShooter getRoller() {
        return roller;
    }

    public void setNeutralMode(NeutralModeValue mode) {
        extender.setMode(mode);
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

    public void setRetractBlockedSupplier(BooleanSupplier retractBlocked) {
        this.retractBlocked = retractBlocked != null ? retractBlocked : () -> false;
    }

    public Command retractIntakeAndWait() {
        return Commands.either(Commands.none(), extender.retractAndWaitCommand(), retractBlocked)
                .withName("RetractIntakeAndWait");
    }

    public Command stowIntake() {
        return Commands.either(Commands.none(), extender.retractCommand(), retractBlocked)
                .withName("StowIntake");
    }

    public Command toggleIntake() {
        return Commands.either(extender.extendCommand(), extender.toggleCommand(), retractBlocked)
                .withName("ToggleIntake");
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
        return Commands.repeatingSequence(
                extender.toggleSiftCommand(), Commands.waitSeconds(0.45).until(extender.isAtTarget()));
    }

    public Command intakeRollerCommand() {
        return roller.spinUpFlywheels(() -> RPM.of(tunableIntakeSpeed.get()))
                .beforeStarting(() -> activelyIntaking = true)
                .finallyDo(() -> {
                    activelyIntaking = false;
                    roller.stop();
                })
                .withName("IntakeRoller");
    }

    public Command outtakeRollerCommand() {
        return roller.spinUpFlywheels(() -> RPM.of(tunableOuttakeSpeed.get())).withName("OuttakeRoller");
    }

    public Command idleRollerCommand() {
        return roller.spinUpFlywheels(() -> RPM.of(tunableIdleSpeed.get())).withName("IdleRoller");
    }

    public Command stopRollerCommand() {
        return roller.stopCommand();
    }

    public boolean isRollerRunning() {
        return roller.isRunning();
    }

    /**
     * True while {@link #intakeRollerCommand()} is running, including when nested inside a Parallel/Sequential group
     * (getCurrentCommand() only shows the outer group name).
     */
    public boolean isIntaking() {
        return activelyIntaking;
    }

    public BooleanSupplier isExtendedSupplier() {
        return extender.isExtended();
    }

    public BooleanSupplier isRollerRunningSupplier() {
        return this::isRollerRunning;
    }

    public BooleanSupplier isActivelyIntakingSupplier() {
        return this::isIntaking;
    }

    public Command intakeCommand() {
        return extender.extendCommand().andThen(intakeRollerCommand());
    }

    public Command retractIntakeCommand() {
        return Commands.either(Commands.none(), roller.stopCommand().andThen(extender.retractCommand()), retractBlocked)
                .withName("RetractIntake");
    }

    public Command outtakeCommand() {
        return extender.extendCommand().andThen(outtakeRollerCommand());
    }
}
