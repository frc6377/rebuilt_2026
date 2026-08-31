package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.extender.Extender;
import frc.robot.subsystems.intake.extender.ExtenderIO;
import frc.robot.subsystems.shooter.BaseShooter;
import frc.robot.subsystems.shooter.BaseShooterIO;
import frc.robot.util.NerfModeController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake {
    private final Extender extender;
    private final BaseShooter roller;
    private final NerfModeController nerfModeController;

    private final LoggedNetworkNumber tunableIntakeSpeed;
    private final LoggedNetworkNumber tunableOuttakeSpeed;
    private final LoggedNetworkNumber tunableIdleSpeed;

    public Intake(ExtenderIO extenderIO, BaseShooterIO rollerIO, NerfModeController nerfModeController) {
        this.nerfModeController = nerfModeController;
        IntakeConstants constants = nerfModeController.getIntakeConstants();
        this.extender = new Extender(extenderIO);
        this.roller = new BaseShooter(rollerIO, constants.rollerConfig(), nerfModeController.getShooterConstants());

        this.tunableIntakeSpeed = new LoggedNetworkNumber(
                "Intake/Roller/IntakeSpeed", constants.rollerIntakeSpeed().in(RPM));
        this.tunableOuttakeSpeed = new LoggedNetworkNumber(
                "Intake/Roller/OuttakeSpeed", constants.rollerOuttakeSpeed().in(RPM));
        this.tunableIdleSpeed = new LoggedNetworkNumber(
                "Intake/Roller/IdleSpeed", constants.rollerIdleSpeed().in(RPM));
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
        return Commands.repeatingSequence(
                extender.toggleSiftCommand(), Commands.waitSeconds(0.45).until(extender.isAtTarget()));
    }

    public Command intakeRollerCommand() {
        return roller.spinUpFlywheels(() -> RPM.of(tunableIntakeSpeed.get())).finallyDo(() -> roller.stop());
    }

    public Command outtakeRollerCommand() {
        return roller.spinUpFlywheels(() -> RPM.of(tunableOuttakeSpeed.get()));
    }

    public Command idleRollerCommand() {
        return roller.spinUpFlywheels(() -> RPM.of(tunableIdleSpeed.get()));
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
        return extender.extendCommand().andThen(intakeRollerCommand());
    }

    public Command retractIntakeCommand() {
        return roller.stopCommand().andThen(extender.retractCommand());
    }

    public Command outtakeCommand() {
        return extender.extendCommand().andThen(outtakeRollerCommand());
    }
}
