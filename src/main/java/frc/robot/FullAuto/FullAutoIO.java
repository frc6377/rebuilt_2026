package frc.robot.FullAuto;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.PathGenerator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public abstract class FullAutoIO extends Command {
    private enum Stage {
        FINDING,
        INTAKING,
        SCORING
    }

    private static final double kIntakeTimeoutSeconds = 12.0;
    private static final double kApproachStandoffMeters = 0.35;
    private static final double kIntakeFuelPercent = 0.9;
    private static final double kVisibleFuelLimit = 8;
    private static final Time kSearchRotateTimeout = Seconds.of(4.5);
    private static final double kMaxValidFuelHeightInches = 4.0;
    private static final double kFuelClusterGapMeters = 0.9;
    private static final double kSearchRotatePercent = 0.1;
    private static final double fuelFindLimit = 10.0;

    private int estimatedFuel = 0;

    protected final Drive drive;
    protected final Intake intake;
    protected final Indexer indexer;
    protected final Superstructure superstructure;
    protected final RobotContainer robot;

    private final Pose2d scoringPose = new Pose2d(2.5, 4.0, Rotation2d.kZero);
    private final Pose2d[] searchPoses = {
        new Pose2d(FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth * 0.1, Rotation2d.fromDegrees(90)),
        new Pose2d(FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth * 0.9, Rotation2d.fromDegrees(-90)),
        new Pose2d(2.5, 4.0, Rotation2d.kZero),
        new Pose2d(FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth / 2.0, new Rotation2d())
    };
    private final PathConstraints intakeConstraints = new PathConstraints(0.5, 0.5, 0.5, 0.5);

    private Stage stage = Stage.FINDING;
    private int searchIndex;
    private Command action;
    private boolean actionRunning;

    protected FullAutoIO(RobotContainer robot) {
        this.robot = robot;
        drive = robot.getDrive();
        intake = robot.getIntake();
        indexer = robot.getIndexer();
        superstructure = robot.getSuperstructure();
        addRequirements(drive, intake.getExtender(), intake.getRoller(), indexer, superstructure);
        Logger.recordOutput("FullAuto/Stage", "None");
    }

    protected abstract Pose3d[] getVisiblePieces();

    protected abstract boolean intakeHasFuel();

    protected abstract boolean consumeIntakeFuel();

    protected abstract int getActualIntakeFuel();

    protected abstract Command scoringFireCommand();

    @Override
    public void initialize() {
        searchIndex = 0;
        setStage(Stage.FINDING);
    }

    @Override
    public void execute() {
        logStatus(getValidFuel());
        switch (stage) {
            case FINDING -> finding();
            case INTAKING -> intaking();
            case SCORING -> scoring();
        }
    }

    @Override
    public void end(boolean interrupted) {
        cancelAction(interrupted);
        drive.stop();
        Logger.recordOutput("FullAuto/Stage", "None");
    }

    private void setStage(Stage next) {
        if (next == Stage.SCORING && estimatedFuel <= 0) {
            next = Stage.FINDING;
        }

        cancelAction(true);
        stage = next;
        Logger.recordOutput("FullAuto/Stage", stage.name());
        switch (stage) {
            case FINDING -> startSearch();
            case INTAKING -> startIntake();
            case SCORING -> startScoring();
        }
    }

    private void finding() {
        if (estimatedFuel > fuelFindLimit) {
            setStage(Stage.SCORING);
            return;
        }

        runAction();
        if (!actionFinished()) {
            return;
        }

        Pose3d[] validFuel = getValidFuel();
        if (enoughVisibleFuel()
                || (validFuel.length > 0 && validFuel.length > IntakeConstants.kIntakeCapacity * kIntakeFuelPercent)) {
            setStage(Stage.INTAKING);
            return;
        }

        searchIndex = (searchIndex + 1) % searchPoses.length;
        startSearch();
    }

    private void startSearch() {
        Pose2d waypoint = searchPoses[searchIndex];
        Logger.recordOutput("FullAuto/SearchPose", waypoint);
        startAction(PathGenerator.pathfindToPose(drive, waypoint)
                .andThen(Commands.run(
                                () -> drive.runVelocity(new ChassisSpeeds(
                                        0, 0, drive.getMaxAngularSpeedRadPerSec() * kSearchRotatePercent)),
                                drive)
                        .until(this::enoughVisibleFuel)
                        .withTimeout(kSearchRotateTimeout))
                .withName("FullAutoSearch"));
    }

    private boolean enoughVisibleFuel() {
        return getValidFuel().length >= kVisibleFuelLimit;
    }

    private void intaking() {
        runAction();
        if (action == null || actionFinished()) {
            if (estimatedFuel > 0) {
                setStage(Stage.SCORING);
            } else if (getValidFuel().length > 0) {
                startIntake();
            } else {
                setStage(Stage.FINDING);
            }
        }
    }

    private void startIntake() {
        Pose3d[] validFuel = getValidFuel();
        Optional<PathPlannerPath> pathOpt = pathFromFuel(validFuel);
        if (pathOpt.isEmpty()) {
            action = null;
            actionRunning = false;
            return;
        }

        PathPlannerPath path = pathOpt.get();
        estimatedFuel += estimateFuelOnPath(path, validFuel);
        Pose2d start = path.getStartingHolonomicPose().orElseThrow();

        startAction(intake.extendIntake()
                .andThen(Commands.parallel(
                        PathGenerator.pathfindToPose(drive, start, intakeConstraints, MetersPerSecond.zero())
                                .andThen(AutoBuilder.followPath(path)),
                        intake.intakeRollerCommand()))
                .withTimeout(kIntakeTimeoutSeconds)
                .withName("FullAutoIntake"));
    }

    private void scoring() {
        runAction();
        if (action == null || actionFinished()) {
            setStage(Stage.FINDING);
        }
    }

    private void startScoring() {
        estimatedFuel = 0;
        startAction(PathGenerator.pathfindToPose(drive, scoringPose)
                .andThen(Commands.parallel(
                                robot.shootAutoCommand(() -> 0.0, () -> 0.0, () -> false), scoringFireCommand())
                        .until(() -> !intakeHasFuel())
                        .withTimeout(15.0))
                .andThen(robot.shootAutoStopCommand().withTimeout(0.25))
                .withName("FullAutoScore"));
    }

    private void logStatus(Pose3d[] validFuel) {
        Logger.recordOutput("FullAuto/ValidVisibleFuel", validFuel.length);
        Logger.recordOutput("FullAuto/EstimatedFuel", estimatedFuel);
        Logger.recordOutput("FullAuto/ActualIntakeFuel", getActualIntakeFuel());
    }

    protected Pose3d[] getValidFuel() {
        Pose3d[] pieces = getVisiblePieces();
        List<Pose3d> valid = new ArrayList<>(pieces.length);
        for (Pose3d piece : pieces) {
            if (piece.getMeasureZ().in(Inches) < kMaxValidFuelHeightInches) {
                valid.add(piece);
            }
        }
        return valid.toArray(Pose3d[]::new);
    }

    private int estimateFuelOnPath(PathPlannerPath path, Pose3d[] visibleFuel) {
        double halfWidth = IntakeConstants.kIntakeWidth.div(2.0).in(Meters);
        int count = 0;
        for (Pose3d pose : visibleFuel) {
            if (distanceToPath(path, pose.toPose2d()) <= halfWidth) {
                count++;
            }
        }
        return count;
    }

    private double distanceToPath(PathPlannerPath path, Pose2d target) {
        double min = Double.MAX_VALUE;
        for (Pose2d pathPose : path.getPathPoses()) {
            min = Math.min(min, pathPose.getTranslation().getDistance(target.getTranslation()));
        }
        return min;
    }

    private Optional<PathPlannerPath> pathFromFuel(Pose3d[] fuel) {
        List<FuelCluster> clusters = getVisibleClusters(fuel);
        if (clusters.isEmpty()) {
            return Optional.empty();
        }

        List<Pose3d> cluster = selectBestCluster(clusters).poses();
        Pose2d start;
        Pose2d end;

        if (cluster.size() == 1) {
            Translation2d p = cluster.get(0).toPose2d().getTranslation();
            Rotation2d heading = p.minus(drive.getPose().getTranslation()).getAngle();
            start = new Pose2d(p.minus(new Translation2d(kApproachStandoffMeters, heading)), heading);
            end = new Pose2d(p, heading);
        } else {
            int n = cluster.size();
            double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0, sumY2 = 0;
            for (Pose3d piece : cluster) {
                sumX += piece.getX();
                sumY += piece.getY();
                sumXY += piece.getX() * piece.getY();
                sumX2 += piece.getX() * piece.getX();
                sumY2 += piece.getY() * piece.getY();
            }

            double denomX = n * sumX2 - sumX * sumX;
            double denomY = n * sumY2 - sumY * sumY;
            Translation2d a;
            Translation2d b;

            if (Math.abs(denomX) >= Math.abs(denomY)) {
                double m = (n * sumXY - sumX * sumY) / denomX;
                double intercept = (sumY - m * sumX) / n;
                double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
                for (Pose3d piece : cluster) {
                    minX = Math.min(minX, piece.getX());
                    maxX = Math.max(maxX, piece.getX());
                }
                a = new Translation2d(minX, m * minX + intercept);
                b = new Translation2d(maxX, m * maxX + intercept);
            } else {
                double m = (n * sumXY - sumX * sumY) / denomY;
                double intercept = (sumX - m * sumY) / n;
                double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
                for (Pose3d piece : cluster) {
                    minY = Math.min(minY, piece.getY());
                    maxY = Math.max(maxY, piece.getY());
                }
                a = new Translation2d(m * minY + intercept, minY);
                b = new Translation2d(m * maxY + intercept, maxY);
            }

            if (a.getDistance(b) < 1e-3) {
                Rotation2d heading = a.minus(drive.getPose().getTranslation()).getAngle();
                start = new Pose2d(a.minus(new Translation2d(kApproachStandoffMeters, heading)), heading);
                end = new Pose2d(a, heading);
            } else {
                Translation2d robotPose = drive.getPose().getTranslation();
                boolean aCloser = a.getDistance(robotPose) <= b.getDistance(robotPose);
                Translation2d from = aCloser ? a : b;
                Translation2d to = aCloser ? b : a;
                Rotation2d heading = to.minus(from).getAngle();
                start = new Pose2d(from.minus(new Translation2d(kApproachStandoffMeters, heading)), heading);
                end = new Pose2d(to, heading);
            }
        }

        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(start, end),
                intakeConstraints,
                new IdealStartingState(0.0, start.getRotation()),
                new GoalEndState(0.0, end.getRotation()));
        path.preventFlipping = true;
        return Optional.of(path);
    }

    private record FuelCluster(int fuelAmount, double distance, List<Pose3d> poses) {}

    private static double distanceFrom(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }

    private List<FuelCluster> getVisibleClusters(Pose3d[] fuel) {
        List<Pose3d> remaining = new ArrayList<>(List.of(fuel));
        List<FuelCluster> clusters = new ArrayList<>();
        Pose2d robotPose = drive.getPose();

        while (!remaining.isEmpty()) {
            int seedIndex = 0;
            for (int i = 1; i < remaining.size(); i++) {
                if (distanceFrom(robotPose, remaining.get(i).toPose2d())
                        < distanceFrom(robotPose, remaining.get(seedIndex).toPose2d())) {
                    seedIndex = i;
                }
            }

            List<Pose3d> poses = new ArrayList<>();
            poses.add(remaining.remove(seedIndex));
            boolean added = true;
            while (added) {
                added = false;
                for (int i = remaining.size() - 1; i >= 0; i--) {
                    Pose3d ball = remaining.get(i);
                    for (Pose3d inCluster : poses) {
                        if (distanceFrom(inCluster.toPose2d(), ball.toPose2d()) <= kFuelClusterGapMeters) {
                            poses.add(remaining.remove(i));
                            added = true;
                            break;
                        }
                    }
                }
            }

            clusters.add(new FuelCluster(
                    poses.size(), distanceFrom(robotPose, poses.get(0).toPose2d()), poses));
        }

        return clusters;
    }

    private FuelCluster selectBestCluster(List<FuelCluster> clusters) {
        FuelCluster best = clusters.get(0);
        for (int i = 1; i < clusters.size(); i++) {
            FuelCluster c = clusters.get(i);
            if (c.fuelAmount() > best.fuelAmount()
                    || (c.fuelAmount() == best.fuelAmount() && c.distance() < best.distance())) {
                best = c;
            }
        }
        return best;
    }

    private void startAction(Command command) {
        cancelAction(true);
        action = command;
        action.initialize();
        actionRunning = true;
    }

    private void runAction() {
        if (!actionRunning || action == null) {
            return;
        }
        action.execute();
        if (action.isFinished()) {
            action.end(false);
            actionRunning = false;
        }
    }

    private boolean actionFinished() {
        return !actionRunning;
    }

    private void cancelAction(boolean interrupted) {
        if (actionRunning && action != null) {
            action.end(interrupted);
        }
        actionRunning = false;
        action = null;
    }
}
