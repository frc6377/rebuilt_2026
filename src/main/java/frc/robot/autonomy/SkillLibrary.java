package frc.robot.autonomy;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.function.BooleanSupplier;

/** Converts task-level objectives into safe, reusable robot skills. */
public class SkillLibrary {
    private final Drive drive;
    private final Superstructure superstructure;
    private final Intake intake;
    private final Indexer indexer;
    private final AutonomyConfig config;

    public SkillLibrary(
            Drive drive, Superstructure superstructure, Intake intake, Indexer indexer, AutonomyConfig config) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.intake = intake;
        this.indexer = indexer;
        this.config = config;
    }

    public Command pathToObjective(AutonomyObjective objective) {
        return AutoBuilder.pathfindToPose(objective.targetPose(), config.pathConstraints())
                .withTimeout(config.pathTimeoutSeconds())
                .andThen(Commands.runOnce(drive::stop, drive))
                .withName("SkillPathTo:" + objective.targetId());
    }

    public Command collectFuel(BooleanSupplier hasPossession) {
        return Commands.deadline(
                        Commands.waitUntil(hasPossession).withTimeout(config.intakeTimeoutSeconds()),
                        intake.intakeCommand(),
                        indexer.index())
                .finallyDo(() -> {
                    intake.getRoller().stop();
                    indexer.setRunning(false);
                })
                .withName("SkillCollectFuel");
    }

    public Command scoreHub(BooleanSupplier readyToScore, Runnable onFeedStarted) {
        return Commands.sequence(
                        superstructure
                                .autonomousAimAndSpinUp(drive)
                                .until(readyToScore)
                                .withTimeout(config.scorePrepareTimeoutSeconds()),
                        Commands.either(
                                Commands.deadline(
                                                Commands.waitSeconds(config.scoreFeedSeconds()),
                                                superstructure.autoSpeedShooter(
                                                        drive::getPose, drive::getChassisSpeeds),
                                                DriveCommands.holdAngle(drive, superstructure::getTargetHeading),
                                                superstructure.fireCommand(),
                                                indexer.index(),
                                                intake.intakeRollerCommand(),
                                                intake.siftFuelCommand())
                                        .beforeStarting(onFeedStarted),
                                Commands.none(),
                                readyToScore))
                .finallyDo(() -> {
                    superstructure.stopUpgoer();
                    indexer.setRunning(false);
                    intake.getRoller().stop();
                })
                .withName("SkillScoreHub");
    }

    public Command relocalize() {
        return Commands.sequence(Commands.runOnce(this::safeStop), Commands.waitSeconds(config.relocalizeSeconds()))
                .withName("SkillRelocalize");
    }

    public Command waitForHub() {
        return Commands.sequence(Commands.runOnce(this::safeStop), Commands.waitSeconds(config.waitForHubSeconds()))
                .withName("SkillWaitForHub");
    }

    public Command recover() {
        return Commands.sequence(Commands.runOnce(this::safeStop), Commands.waitSeconds(config.recoverSeconds()))
                .withName("SkillRecover");
    }

    public void safeStop() {
        drive.stop();
        drive.stopWithX();
        superstructure.stopUpgoer();
        superstructure.stopShooter();
        indexer.setRunning(false);
        intake.getRoller().stop();
        intake.getExtender().setMotorPercentage(0.0);
    }
}
