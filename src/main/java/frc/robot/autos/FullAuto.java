package frc.robot.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.PathGenerator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.GamePieceTrajectorySimulation;
import frc.robot.subsystems.superstructure.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.GamePieceCameraSim;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class FullAuto extends Command {
    private enum Stage {
        FINDING,
        INTAKING,
        SCORING
    }

    private final Drive drive;
    private final Intake intake;
    private final Indexer indexer;
    private final Superstructure superstructure;
    private final GamePieceCameraSim camera;
    private final RobotContainer robot;

    private final Pose2d scoringPose = new Pose2d(2.5, 4.0, Rotation2d.kZero);

    private final Pose2d[] searchPoses = {
        new Pose2d(FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth / 2.0, Rotation2d.kZero), scoringPose
    };
    private final PathConstraints intakeConstraints = new PathConstraints(0.5, 0.5, 0.5, 0.5);

    private Stage stage = Stage.FINDING;
    private int searchIndex;
    private Command action;
    private boolean actionRunning;

    public FullAuto(RobotContainer robot) {
        this.robot = robot;
        drive = robot.getDrive();
        intake = robot.getIntake();
        indexer = robot.getIndexer();
        superstructure = robot.getSuperstructure();
        camera = robot.getGamePieceCamera();
        addRequirements(drive, intake.getExtender(), intake.getRoller(), indexer, superstructure);
        Logger.recordOutput("FullAuto/Stage", "None");
    }

    @Override
    public void initialize() {
        searchIndex = 0;
        setStage(Stage.FINDING);
    }

    @Override
    public void execute() {
        switch (stage) {
            case FINDING -> finding();
            case INTAKING -> intaking();
            case SCORING -> scoring();
        }
    }

    @Override
    public void end(boolean interrupted) {
        cancelAction();
        drive.stop();
    }

    private void setStage(Stage next) {
        cancelAction();
        stage = next;
        Logger.recordOutput("FullAuto/Stage", stage.name());
        switch (stage) {
            case FINDING -> startSearch();
            case INTAKING -> startIntake();
            case SCORING -> startScoring();
        }
    }

    private void finding() {
        if (seesFuel()) {
            setStage(Stage.INTAKING);
            return;
        }
        runAction();
        if (!actionFinished()) return;

        searchIndex = (searchIndex + 1) % searchPoses.length;
        startSearch();
    }

    private void startSearch() {
        double rotatePercent = 0.3;
        startAction(PathGenerator.pathfindToPose(drive, searchPoses[searchIndex])
                .andThen(Commands.run(
                                () -> drive.runVelocity(
                                        new ChassisSpeeds(0, 0, drive.getMaxAngularSpeedRadPerSec() * rotatePercent)),
                                drive)
                        .until(this::seesFuel)));
    }

    private void intaking() {
        if (intakeHasGamePiece()) {
            setStage(Stage.SCORING);
            return;
        }
        if (action == null || actionFinished()) {
            setStage(Stage.FINDING);
            return;
        }
        runAction();
    }

    private void startIntake() {
        Optional<Pose3d> fuel = closestFuel();
        if (fuel.isEmpty()) {
            action = null;
            actionRunning = false;
            return;
        }
        Pose2d pose = fuel.get().toPose2d();
        startAction(intake.extendIntake()
                .andThen(Commands.deadline(
                        PathGenerator.pathfindToPose(
                                drive,
                                new Pose2d(pose.getTranslation(), drive.getRotation()),
                                intakeConstraints,
                                MetersPerSecond.zero()),
                        intake.intakeRollerCommand())));
    }

    private void scoring() {
        logTrajectory();
        runAction();
        if (actionFinished()) {
            setStage(Stage.FINDING);
        }
    }

    private void startScoring() {
        Logger.recordOutput("Full Auto/ScoringPose", scoringPose);
        startAction(PathGenerator.pathfindToPose(drive, scoringPose)
                .andThen(Commands.runOnce(this::transferIntakeFuelToHopper))
                .andThen(Commands.parallel(
                                robot.shootAutoCommand(() -> 0.0, () -> 0.0, () -> false),
                                superstructure.simAutoFireHoldCommand(() -> true),
                                Commands.run(this::logTrajectory))
                        .withTimeout(3.0))
                .andThen(robot.shootAutoStopCommand().withTimeout(0.25)));
    }

    /** Move sim intake fuel into the hopper so GamePieceTrajectorySimulation can eject it. */
    private void transferIntakeFuelToHopper() {
        GamePieceTrajectorySimulation traj = superstructure.getGamePieceTrajectorySimulation();
        var intakeSim = robot.getIntakeSimulation();
        if (traj == null || intakeSim == null) {
            return;
        }

        int transferred = 0;
        while (intakeSim.obtainGamePieceFromIntake()) {
            transferred++;
        }
        if (transferred > 0) {
            traj.addBalls(transferred);
            RobotState state = RobotState.getInstance();
            if (state != null) {
                state.setSimGamePieceCount(traj.getBallsInHopper());
            }
        }
        Logger.recordOutput("FullAuto/TransferredFuel", transferred);
        Logger.recordOutput("FullAuto/HopperBalls", traj.getBallsInHopper());
    }

    /** Preview and log the current shot trajectory (AdvantageScope Pose3d[]). */
    private void logTrajectory() {
        GamePieceTrajectorySimulation traj = superstructure.getGamePieceTrajectorySimulation();
        if (traj == null) {
            return;
        }

        Logger.recordOutput("FullAuto/PreviewTrajectory", traj.previewTrajectory());
        Logger.recordOutput("FullAuto/PredictedLanding", traj.getPredictedLandingPosition());
        Logger.recordOutput("FullAuto/HorizontalRangeMeters", traj.getHorizontalRange());
        Logger.recordOutput("FullAuto/HopperBalls", traj.getBallsInHopper());
    }

    private boolean intakeHasGamePiece() {
        var intakeSim = robot.getIntakeSimulation();
        return intakeSim != null && intakeSim.getGamePiecesAmount() > 0;
    }

    private boolean seesFuel() {
        return camera != null && camera.getVisiblePieces().length != 0;
    }

    private Optional<Pose3d> closestFuel() {
        return camera == null ? Optional.empty() : camera.getClosestVisiblePiece();
    }

    private void startAction(Command command) {
        cancelAction();
        action = command;
        action.initialize();
        actionRunning = true;
    }

    private void runAction() {
        if (actionRunning) action.execute();
    }

    private boolean actionFinished() {
        return !actionRunning || action.isFinished();
    }

    private void cancelAction() {
        if (actionRunning) {
            action.end(true);
            actionRunning = false;
        }
        action = null;
    }
}
