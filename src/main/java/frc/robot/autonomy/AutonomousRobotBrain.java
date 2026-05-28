package frc.robot.autonomy;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/** Autonomy-first robot brain that plans objectives, executes skills, and adapts after failures. */
public class AutonomousRobotBrain extends Command {
    private final Drive drive;
    private final Vision vision;
    private final Superstructure superstructure;
    private final Intake intake;
    private final Indexer indexer;
    private final RobotState robotState;
    private final AutonomyConfig config;
    private final ObjectivePlanner planner = new ObjectivePlanner();
    private final WorldModel worldModel = new WorldModel();
    private final SkillLibrary skills;
    private final boolean finishAfterOneScoredCycle;

    private AutonomyState state = AutonomyState.DISARMED;
    private AutonomyObjective activeObjective = AutonomyObjective.stop("Disarmed");
    private Command activeCommand = null;
    private boolean activeCommandWasScheduled = false;
    private double activeCommandRequestedTimestamp = 0.0;
    private double stateStartTimestamp = 0.0;
    private boolean timedPossessionFallback = false;
    private boolean scoreFeedStarted = false;
    private int recoveryAttempts = 0;
    private int completedScoredCycles = 0;
    private boolean finishRequested = false;
    private String lastFault = "None";

    public AutonomousRobotBrain(
            Drive drive,
            Vision vision,
            Superstructure superstructure,
            Intake intake,
            Indexer indexer,
            RobotState robotState,
            AutonomyConfig config,
            boolean finishAfterOneScoredCycle) {
        this.drive = drive;
        this.vision = vision;
        this.superstructure = superstructure;
        this.intake = intake;
        this.indexer = indexer;
        this.robotState = robotState;
        this.config = config;
        this.finishAfterOneScoredCycle = finishAfterOneScoredCycle;
        skills = new SkillLibrary(drive, superstructure, intake, indexer, config);
        setName(finishAfterOneScoredCycle ? "AutonomousBrainOneCycle" : "AutonomousBrainContinuous");
    }

    @Override
    public void initialize() {
        planner.reset();
        worldModel.reset();
        activeCommand = null;
        activeCommandWasScheduled = false;
        activeObjective = AutonomyObjective.stop("Starting");
        timedPossessionFallback = robotState.getSimGamePieceCount() > 0;
        scoreFeedStarted = false;
        recoveryAttempts = 0;
        completedScoredCycles = 0;
        finishRequested = false;
        lastFault = "None";
        transition(AutonomyState.SELECT_OBJECTIVE);
    }

    @Override
    public void execute() {
        worldModel.update(drive, vision, robotState, superstructure, config, timedPossessionFallback);
        log();

        if (DriverStation.isDisabled()) {
            fault("RobotDisabled");
            transition(AutonomyState.DISABLED);
            finishRequested = true;
            return;
        }

        switch (state) {
            case SELECT_OBJECTIVE -> selectObjective();
            case PATH_TO_OBJECTIVE -> runPathToObjective();
            case EXECUTE_OBJECTIVE -> runObjectiveSkill();
            case RECOVER -> runRecovery();
            case DISABLED, DISARMED, FAULTED -> finishRequested = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        cancelActiveCommand();
        skills.safeStop();
        Logger.recordOutput("Autonomy/Brain/Interrupted", interrupted);
    }

    @Override
    public boolean isFinished() {
        return finishRequested;
    }

    private void selectObjective() {
        activeObjective = planner.select(worldModel.snapshot(), config);
        Logger.recordOutput(
                "Autonomy/Brain/SelectedObjectiveType", activeObjective.type().toString());
        Logger.recordOutput("Autonomy/Brain/SelectedObjectiveId", activeObjective.targetId());
        Logger.recordOutput("Autonomy/Brain/SelectedObjectiveScore", activeObjective.score());
        Logger.recordOutput("Autonomy/Brain/SelectedObjectivePose", activeObjective.targetPose());

        switch (activeObjective.type()) {
            case COLLECT_KNOWN_FUEL, COLLECT_VISIBLE_FUEL, SCORE_HUB -> transition(AutonomyState.PATH_TO_OBJECTIVE);
            case RELOCALIZE, WAIT_FOR_HUB -> transition(AutonomyState.EXECUTE_OBJECTIVE);
            case RECOVER -> transition(AutonomyState.RECOVER);
            case STOP -> {
                transition(AutonomyState.DISARMED);
                finishRequested = true;
            }
        }
    }

    private void runPathToObjective() {
        if (activeCommand == null) {
            startChild(skills.pathToObjective(activeObjective));
            return;
        }
        if (!childComplete()) {
            return;
        }
        if (nearObjective()) {
            transition(AutonomyState.EXECUTE_OBJECTIVE);
        } else {
            failObjective("PathMissedTarget");
            transition(AutonomyState.RECOVER);
        }
    }

    private void runObjectiveSkill() {
        if (activeCommand == null) {
            startChild(skillForObjective(activeObjective));
            return;
        }
        if (!childComplete()) {
            return;
        }

        switch (activeObjective.type()) {
            case COLLECT_KNOWN_FUEL, COLLECT_VISIBLE_FUEL -> finishCollectObjective();
            case SCORE_HUB -> finishScoreObjective();
            case RELOCALIZE, WAIT_FOR_HUB -> transition(AutonomyState.SELECT_OBJECTIVE);
            case RECOVER -> transition(AutonomyState.RECOVER);
            case STOP -> finishRequested = true;
        }
    }

    private Command skillForObjective(AutonomyObjective objective) {
        return switch (objective.type()) {
            case COLLECT_KNOWN_FUEL, COLLECT_VISIBLE_FUEL -> skills.collectFuel(
                    () -> worldModel.snapshot().hasPossession());
            case SCORE_HUB -> {
                scoreFeedStarted = false;
                yield skills.scoreHub(
                        () -> superstructure.isReadyToShoot(drive.getRotation())
                                && worldModel.snapshot().scoringAllowed(),
                        () -> scoreFeedStarted = true);
            }
            case RELOCALIZE -> skills.relocalize();
            case WAIT_FOR_HUB -> skills.waitForHub();
            case RECOVER -> skills.recover();
            case STOP -> skills.recover();
        };
    }

    private void finishCollectObjective() {
        timedPossessionFallback = true;
        if (Constants.currentMode == Constants.Mode.SIM && robotState.getSimGamePieceCount() == 0) {
            robotState.incrementSimGamePieceCount();
        }
        recoveryAttempts = 0;
        transition(AutonomyState.SELECT_OBJECTIVE);
    }

    private void finishScoreObjective() {
        if (!scoreFeedStarted || !worldModel.snapshot().hubActive()) {
            failObjective(worldModel.snapshot().hubActive() ? "ScoreNotReadyAfterAttempt" : "HubInactiveAfterAttempt");
            transition(AutonomyState.RECOVER);
            return;
        }
        scoreFeedStarted = false;
        timedPossessionFallback = false;
        if (Constants.currentMode == Constants.Mode.SIM) {
            robotState.decrementSimGamePieceCount();
        }
        completedScoredCycles++;
        recoveryAttempts = 0;
        if (finishAfterOneScoredCycle) {
            finishRequested = true;
        } else {
            transition(AutonomyState.SELECT_OBJECTIVE);
        }
    }

    private void runRecovery() {
        if (activeCommand == null) {
            recoveryAttempts++;
            if (recoveryAttempts > config.maxRecoveryAttempts()) {
                fault("RecoveryAttemptsExceeded");
                transition(AutonomyState.FAULTED);
                finishRequested = true;
                return;
            }
            startChild(skills.recover());
            return;
        }
        if (childComplete()) {
            transition(AutonomyState.SELECT_OBJECTIVE);
        }
    }

    private boolean nearObjective() {
        return worldModel
                        .snapshot()
                        .robotPose()
                        .getTranslation()
                        .getDistance(activeObjective.targetPose().getTranslation())
                <= config.goalToleranceMeters();
    }

    private void startChild(Command command) {
        activeCommand = command;
        activeCommandWasScheduled = false;
        activeCommandRequestedTimestamp = Timer.getFPGATimestamp();
        CommandScheduler.getInstance().schedule(activeCommand);
    }

    private boolean childComplete() {
        if (activeCommand == null) {
            return true;
        }
        boolean scheduled = CommandScheduler.getInstance().isScheduled(activeCommand);
        if (scheduled) {
            activeCommandWasScheduled = true;
            return false;
        }
        if (!activeCommandWasScheduled && Timer.getFPGATimestamp() - activeCommandRequestedTimestamp < 0.1) {
            return false;
        }
        activeCommand = null;
        return true;
    }

    private void transition(AutonomyState nextState) {
        if (state == nextState) {
            return;
        }
        cancelActiveCommand();
        state = nextState;
        stateStartTimestamp = Timer.getFPGATimestamp();
        Logger.recordOutput("Autonomy/Brain/StateTransition", nextState.toString());
    }

    private void cancelActiveCommand() {
        if (activeCommand != null && CommandScheduler.getInstance().isScheduled(activeCommand)) {
            CommandScheduler.getInstance().cancel(activeCommand);
        }
        activeCommand = null;
        activeCommandWasScheduled = false;
    }

    private void failObjective(String fault) {
        planner.markFailed(activeObjective, config);
        fault(fault + ":" + activeObjective.targetId());
    }

    private void fault(String fault) {
        lastFault = fault;
        Logger.recordOutput("Autonomy/Brain/LastFault", lastFault);
    }

    private void log() {
        Logger.recordOutput("Autonomy/Brain/State", state.toString());
        Logger.recordOutput("Autonomy/Brain/StateElapsedSeconds", Timer.getFPGATimestamp() - stateStartTimestamp);
        Logger.recordOutput("Autonomy/Brain/ActiveCommand", activeCommand == null ? "None" : activeCommand.getName());
        Logger.recordOutput(
                "Autonomy/Brain/ActiveObjectiveType", activeObjective.type().toString());
        Logger.recordOutput("Autonomy/Brain/ActiveObjectiveId", activeObjective.targetId());
        Logger.recordOutput("Autonomy/Brain/RecoveryAttempts", recoveryAttempts);
        Logger.recordOutput("Autonomy/Brain/CompletedScoredCycles", completedScoredCycles);
        Logger.recordOutput("Autonomy/Brain/TimedPossessionFallback", timedPossessionFallback);
        Logger.recordOutput("Autonomy/Brain/LastFault", lastFault);
        Logger.recordOutput("Autonomy/Brain/ObjectiveScoringModel", planner.scoringModelName());
        worldModel.log();
    }
}
