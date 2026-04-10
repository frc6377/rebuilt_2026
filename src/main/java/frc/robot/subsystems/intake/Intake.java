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

    private final LoggedNetworkNumber tunableIntakeSpeed =
            new LoggedNetworkNumber("Intake/Roller/IntakeSpeed", IntakeConstants.RollerConstants.kIntakeSpeed.in(RPM));
    private final LoggedNetworkNumber tunableOuttakeSpeed = new LoggedNetworkNumber(
            "Intake/Roller/OuttakeSpeed", IntakeConstants.RollerConstants.kOuttakeSpeed.in(RPM));
    private final LoggedNetworkNumber tunableIdleSpeed =
            new LoggedNetworkNumber("Intake/Roller/IdleSpeed", IntakeConstants.RollerConstants.kIdleSpeed.in(RPM));

    public Intake(ExtenderIO extenderIO, BaseShooterIO rollerIO) {
        this.extender = new Extender(extenderIO);
        this.roller = new BaseShooter(rollerIO, IntakeConstants.RollerConstants.rollerConfig);

        //        this.roller.setDefaultCommand(this.roller
        //                .spinUpFlywheels(() ->
        //                        IntakeConstants.RollerConstants.kIdleEnabled ? RPM.of(tunableIdleSpeed.get()) :
        // RPM.of(0.0))
        //                .withName("RollerIdle"));

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
        return Commands.repeatingSequence(extender.toggleSiftCommand(), Commands.waitSeconds(0.5));
    }

    public Command intakeRollerCommand() {
        return roller.spinUpFlywheels(() -> RPM.of(tunableIntakeSpeed.get()));
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
