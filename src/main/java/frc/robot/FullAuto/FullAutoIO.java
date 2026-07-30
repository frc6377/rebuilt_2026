package frc.robot.FullAuto;

import static edu.wpi.first.units.Units.*;

import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public abstract class FullAutoIO extends Command {
    private enum Stage {
        FINDING,
        INTAKING,
        SCORING
    }

    // --- Configuration Constants ---
    private static final double kIntakeTimeoutSeconds = 12.0;
    private static final double kMaxIntakeWindowSeconds = 10.0;
    private static final double kApproachStandoffMeters = 0.35;
    private static final double kOvershootMeters = 0.50;
    private static final double kFieldMarginMeters = 0.50; // Safety buffer to prevent driving into walls
    private static final double kIntakeFuelPercent = 0.9;
    private static final double kVisibleFuelLimit = 8;
    private static final int kMinTrackedFuelToStart = 3;
    private static final Time kSearchRotateTimeout = Seconds.of(4.5);
    private static final double kMaxValidFuelHeightInches = 4.0;
    private static final double kFuelClusterGapMeters = 0.9;
    private static final double kSearchRotatePercent = 0.25;
    private static final double kFuelFindLimit = 10.0;
    private static final double kFuelClearDistanceMeters = 0.4;
    private static final double kHubExclusionRadiusMeters = 0;

    // --- NEW: Stuck Detection Constants ---
    private static final double kStuckTimeWindowSeconds = 1.0;
    private static final double kStuckDistanceThresholdMeters = 0.05;

    // Extracted Magic Numbers
    private static final double kPruneMaxDistanceMeters = 2.0;
    private static final double kPruneFOVAngleDegrees = 45.0;
    private static final double kAutopilotVelocity = 1;
    private static final double kScoringTimeoutSeconds = 10.0;
    private static final double kShootStopTimeSeconds = 0.25;

    // --- State Variables ---
    private int estimatedFuel = 0;
    private double intakeStageStartTime = 0.0;
    private double totalSpinAngleRad = 0.0;
    private Rotation2d lastSpinRotation = new Rotation2d();
    private boolean isSpinSearching = false;

    // --- NEW: Stuck Detection Variables ---
    private double lastStuckCheckTime = 0.0;
    private Pose2d lastStuckCheckPose = new Pose2d();

    protected final Drive drive;
    protected final Intake intake;
    protected final Indexer indexer;
    protected final Superstructure superstructure;
    protected final RobotContainer robot;

    private final Pose2d scoringPose = new Pose2d(2.5, 4.0, Rotation2d.kZero);

    // Search poses restricted strictly to the center line of the Neutral Zone
    private final Pose2d[] searchPoses = {
        new Pose2d(FieldConstants.LinesVertical.center, FieldConstants.fieldWidth * 0.2, Rotation2d.fromDegrees(90)),
        new Pose2d(FieldConstants.LinesVertical.center, FieldConstants.fieldWidth * 0.8, Rotation2d.fromDegrees(-90)),
        new Pose2d(FieldConstants.LinesVertical.center, FieldConstants.fieldWidth * 0.5, Rotation2d.kZero)
    };

    private Stage stage = Stage.FINDING;
    private int searchIndex;
    private Command action;
    private boolean actionRunning;

    private final List<Pose3d> trackedFuelMap = new ArrayList<>();

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
        estimatedFuel = 0;
        trackedFuelMap.clear();
        setStage(Stage.FINDING);
    }

    @Override
    public void execute() {
        Pose3d[] visibleFuel = getValidFuel();
        updateFuelMemory(visibleFuel);
        pruneClearedFuel(visibleFuel);
        logStatus(visibleFuel);

        // 1. Evaluate logic and determine if a state transition/new action is needed
        switch (stage) {
            case FINDING -> finding();
            case INTAKING -> intaking(trackedFuelMap.toArray(new Pose3d[0]));
            case SCORING -> scoring();
        }

        // 2. Centralized Execution: Tick the active action (if any) immediately
        runAction();
    }

    @Override
    public void end(boolean interrupted) {
        cancelAction(interrupted);
        drive.stop();
        Logger.recordOutput("FullAuto/Stage", "None");
        Logger.recordOutput("FullAuto/IntakeStatus", "Ended");
    }

    private void setStage(Stage next) {
        cancelAction(true);
        stage = next;
        isSpinSearching = false;
        Logger.recordOutput("FullAuto/Stage", stage.name());
        Logger.recordOutput("FullAuto/IntakeStatus", "Running normally");

        switch (stage) {
            case FINDING -> startSearch();
            case INTAKING -> {
                intakeStageStartTime = Timer.getFPGATimestamp();
                if (enoughVisibleFuel() || trackedFuelMap.size() >= kMinTrackedFuelToStart) {
                    startIntake();
                }
            }
            case SCORING -> startScoring();
        }
    }

    private void finding() {
        if (hasAnyFuel() && estimatedFuel > kFuelFindLimit) {
            setStage(Stage.SCORING);
            return;
        }

        if (enoughVisibleFuel() || trackedFuelMap.size() >= kMinTrackedFuelToStart) {
            setStage(Stage.INTAKING);
            return;
        }

        if (actionFinished()) {
            searchIndex = (searchIndex + 1) % searchPoses.length;
            startSearch();
        }
    }

    private void startSearch() {
        Pose2d waypoint;

        if (trackedFuelMap.size() >= kMinTrackedFuelToStart) {
            double meanX = trackedFuelMap.stream()
                    .mapToDouble(p -> p.toPose2d().getTranslation().getX())
                    .average()
                    .orElse(0.0);
            double meanY = trackedFuelMap.stream()
                    .mapToDouble(p -> p.toPose2d().getTranslation().getY())
                    .average()
                    .orElse(0.0);

            Translation2d meanPos = clampToFieldBounds(new Translation2d(meanX, meanY));
            Rotation2d heading = meanPos.minus(drive.getPose().getTranslation()).getAngle();
            waypoint = new Pose2d(meanPos, heading);
        } else {
            waypoint = FieldConstants.toCurrentAlliancePose(searchPoses[searchIndex]);
        }

        Logger.recordOutput("FullAuto/SearchPose", waypoint);
        startAction(drive.smartAlign(waypoint).withName("FullAutoSearch"));
    }

    private void updateFuelMemory(Pose3d[] validFuel) {
        for (Pose3d p : validFuel) {
            Translation2d pos = p.toPose2d().getTranslation();
            if (isNearHub(pos)) continue;

            boolean isAlreadyTracked = trackedFuelMap.stream()
                    .anyMatch(
                            tracked -> tracked.toPose2d().getTranslation().getDistance(pos) < kFuelClearDistanceMeters);

            if (!isAlreadyTracked) {
                trackedFuelMap.add(p);
            }
        }
    }

    private void pruneClearedFuel(Pose3d[] currentlyVisible) {
        Pose2d robotPose = drive.getPose();
        Translation2d robotTrans = robotPose.getTranslation();

        Iterator<Pose3d> iterator = trackedFuelMap.iterator();
        while (iterator.hasNext()) {
            Pose3d tracked = iterator.next();

            if (tracked.toPose2d().getTranslation().getDistance(robotTrans) < kFuelClearDistanceMeters) {
                iterator.remove();
                continue;
            }

            boolean sawInVision = Arrays.stream(currentlyVisible)
                    .anyMatch(vis -> vis.toPose2d()
                                    .getTranslation()
                                    .getDistance(tracked.toPose2d().getTranslation())
                            < kFuelClearDistanceMeters);

            if (!sawInVision && tracked.toPose2d().getTranslation().getDistance(robotTrans) < kPruneMaxDistanceMeters) {
                Rotation2d angleToTarget =
                        tracked.toPose2d().getTranslation().minus(robotTrans).getAngle();
                if (Math.abs(angleToTarget.minus(robotPose.getRotation()).getDegrees()) < kPruneFOVAngleDegrees) {
                    iterator.remove();
                }
            }
        }
    }

    private boolean enoughVisibleFuel() {
        return getValidFuel().length >= kVisibleFuelLimit;
    }

    private boolean hasAnyFuel() {
        return estimatedFuel > 0 || getActualIntakeFuel() > 0 || intakeHasFuel();
    }

    private void intaking(Pose3d[] visibleFuel) {
        boolean isHopperFull = estimatedFuel >= IntakeConstants.kIntakeCapacity
                || getActualIntakeFuel() >= IntakeConstants.kIntakeCapacity;
        boolean isTimeUp = (Timer.getFPGATimestamp() - intakeStageStartTime) >= kMaxIntakeWindowSeconds;

        if (isHopperFull || isTimeUp) {
            setStage(hasAnyFuel() ? Stage.SCORING : Stage.FINDING);
            return;
        }

        if (actionRunning && action != null && !isSpinSearching) {
            // --- NEW: Check if we are stuck ---
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - lastStuckCheckTime > kStuckTimeWindowSeconds) {
                double distanceMoved =
                        drive.getPose().getTranslation().getDistance(lastStuckCheckPose.getTranslation());

                // If we haven't moved at least 5cm in the last 1.0 seconds, we hit a wall
                if (distanceMoved < kStuckDistanceThresholdMeters) {
                    Logger.recordOutput("FullAuto/IntakeStatus", "STUCK! Bailing early.");
                    setStage(hasAnyFuel() ? Stage.SCORING : Stage.FINDING);
                    return;
                }

                // Reset anchor for the next check
                lastStuckCheckTime = currentTime;
                lastStuckCheckPose = drive.getPose();
            }
            return;
        }

        if (visibleFuel.length >= kVisibleFuelLimit) {
            isSpinSearching = false;
            if (!startIntake()) setStage(hasAnyFuel() ? Stage.SCORING : Stage.FINDING);
            return;
        }

        handleSpinSearch(visibleFuel);
    }

    private void handleSpinSearch(Pose3d[] visibleFuel) {
        if (!isSpinSearching) {
            isSpinSearching = true;
            totalSpinAngleRad = 0.0;
            lastSpinRotation = drive.getPose().getRotation();

            startAction(intake.intakeRollerCommand());
        }

        drive.runVelocity(new ChassisSpeeds(0, 0, drive.getMaxAngularSpeedRadPerSec() * kSearchRotatePercent));

        Rotation2d currentRot = drive.getPose().getRotation();
        totalSpinAngleRad += Math.abs(currentRot.minus(lastSpinRotation).getRadians());
        lastSpinRotation = currentRot;

        if (totalSpinAngleRad >= (2.0 * Math.PI - 0.2)) {
            isSpinSearching = false;
            drive.stop();
            cancelAction(true);

            if (visibleFuel.length > 0 || trackedFuelMap.size() >= kMinTrackedFuelToStart) {
                if (!startIntake()) {
                    if (!trackedFuelMap.isEmpty()) trackedFuelMap.remove(0);
                    setStage(hasAnyFuel() ? Stage.SCORING : Stage.FINDING);
                }
            } else {
                setStage(hasAnyFuel() ? Stage.SCORING : Stage.FINDING);
            }
        }
    }

    private boolean startIntake() {
        Supplier<APTarget> targetSupplier = () -> {
            Pose3d[] validFuel = getValidFuel();
            List<FuelCluster> clusters = getVisibleClusters(validFuel);

            Translation2d robotTrans = drive.getPose().getTranslation();
            Translation2d targetTrans;
            Rotation2d heading;

            if (!clusters.isEmpty()) {
                FuelCluster bestCluster = selectBestCluster(clusters);
                List<Pose3d> poses = bestCluster.poses();

                double meanX =
                        poses.stream().mapToDouble(Pose3d::getX).average().orElse(robotTrans.getX());
                double meanY =
                        poses.stream().mapToDouble(Pose3d::getY).average().orElse(robotTrans.getY());
                Translation2d centroid = new Translation2d(meanX, meanY);

                double radius = poses.stream()
                        .mapToDouble(p -> p.toPose2d().getTranslation().getDistance(centroid))
                        .max()
                        .orElse(0.0);

                heading = centroid.minus(robotTrans).getAngle();
                double totalPushThroughDistance = radius + kOvershootMeters;

                // Overshoot calculation safely clamped within field boundary limits
                targetTrans = clampToFieldBounds(centroid.plus(new Translation2d(totalPushThroughDistance, heading)));

            } else if (trackedFuelMap.size() >= kMinTrackedFuelToStart) {
                Pose3d tracked = trackedFuelMap.get(0);
                targetTrans = clampToFieldBounds(tracked.toPose2d().getTranslation());
                heading = targetTrans.minus(robotTrans).getAngle();
            } else {
                return new APTarget(drive.getPose());
            }

            Pose2d targetPose = new Pose2d(targetTrans, heading);
            return new APTarget(targetPose).withVelocity(kAutopilotVelocity).withEntryAngle(heading);
        };

        if (targetSupplier.get().getReference().equals(drive.getPose())) {
            cancelAction(true);
            return false;
        }

        estimatedFuel += 1;

        // --- NEW: Reset Stuck Detector when starting a new swoop ---
        lastStuckCheckTime = Timer.getFPGATimestamp();
        lastStuckCheckPose = drive.getPose();
        Logger.recordOutput("FullAuto/IntakeStatus", "Swooping");

        startAction(intake.extendIntake()
                .andThen(Commands.parallel(drive.swoop(targetSupplier), intake.intakeRollerCommand()))
                .withDeadline(Commands.waitUntil(() -> getActualIntakeFuel() > 55))
                .withTimeout(20)
                .withName("FullAutoIntakeDynamicSwoop"));
        return true;
    }

    private void scoring() {
        if (actionFinished()) {
            estimatedFuel = 0;
            trackedFuelMap.clear();
            setStage(Stage.FINDING);
        }
    }

    private void startScoring() {
        startAction(drive.smartAlign(FieldConstants.toCurrentAlliancePose(scoringPose))
                .andThen(Commands.waitSeconds(0.4))
                .andThen(Commands.parallel(robot.shootAutoModeCommand(drive), scoringFireCommand())
                        .until(() -> getActualIntakeFuel() == 0)
                        .andThen(robot.shootAutoStopCommand().withTimeout(kShootStopTimeSeconds))
                        .finallyDo(() -> estimatedFuel = 0)
                        .withName("FullAutoScore")));
    }

    private void logStatus(Pose3d[] validFuel) {
        Logger.recordOutput("FullAuto/ValidVisibleFuel", validFuel.length);
        Logger.recordOutput("FullAuto/EstimatedFuel", estimatedFuel);
        Logger.recordOutput("FullAuto/ActualIntakeFuel", getActualIntakeFuel());
        Logger.recordOutput("FullAuto/TrackedFuelCount", trackedFuelMap.size());
        Logger.recordOutput("FullAuto/TrackedFuelLocations", trackedFuelMap.toArray(new Pose3d[0]));
    }

    protected Pose3d[] getValidFuel() {
        return Arrays.stream(getVisiblePieces())
                .filter(piece -> piece.getMeasureZ().in(Inches) < kMaxValidFuelHeightInches)
                .filter(piece -> !isNearHub(piece.toPose2d().getTranslation()))
                .filter(piece -> isInNeutralZone(piece.toPose2d().getTranslation()))
                .toArray(Pose3d[]::new);
    }

    // Ensures point is inside neutral zone and safely away from side guardrails
    private boolean isInNeutralZone(Translation2d point) {
        double x = point.getX();
        double y = point.getY();
        double minX =
                Math.min(FieldConstants.LinesVertical.neutralZoneNear, FieldConstants.LinesVertical.neutralZoneFar)
                        + kFieldMarginMeters;
        double maxX =
                Math.max(FieldConstants.LinesVertical.neutralZoneNear, FieldConstants.LinesVertical.neutralZoneFar)
                        - kFieldMarginMeters;

        boolean insideX = x >= minX && x <= maxX;
        boolean insideY = y >= kFieldMarginMeters && y <= (FieldConstants.fieldWidth - kFieldMarginMeters);

        return insideX && insideY;
    }

    // Clamps target positions to safe field boundaries
    private Translation2d clampToFieldBounds(Translation2d point) {
        double minX =
                Math.min(FieldConstants.LinesVertical.neutralZoneNear, FieldConstants.LinesVertical.neutralZoneFar)
                        + kFieldMarginMeters;
        double maxX =
                Math.max(FieldConstants.LinesVertical.neutralZoneNear, FieldConstants.LinesVertical.neutralZoneFar)
                        - kFieldMarginMeters;

        double clampedX = Math.max(minX, Math.min(maxX, point.getX()));
        double clampedY =
                Math.max(kFieldMarginMeters, Math.min(FieldConstants.fieldWidth - kFieldMarginMeters, point.getY()));

        return new Translation2d(clampedX, clampedY);
    }

    private boolean isNearHub(Translation2d point) {
        Translation2d hubCenter =
                new Translation2d(FieldConstants.LinesVertical.hubCenter, FieldConstants.fieldWidth / 2.0);
        return point.getDistance(hubCenter) <= kHubExclusionRadiusMeters;
    }

    private record FuelCluster(int fuelAmount, double distance, List<Pose3d> poses) {}

    private static double distanceFrom(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }

    private List<FuelCluster> getVisibleClusters(Pose3d[] fuel) {
        List<Pose3d> remaining = new ArrayList<>(Arrays.asList(fuel));
        List<FuelCluster> clusters = new ArrayList<>();
        Pose2d robotPose = drive.getPose();

        while (!remaining.isEmpty()) {
            Pose3d seed = remaining.stream()
                    .min(Comparator.comparingDouble(p -> distanceFrom(robotPose, p.toPose2d())))
                    .orElse(remaining.get(0));
            remaining.remove(seed);

            List<Pose3d> poses = new ArrayList<>();
            poses.add(seed);

            boolean added = true;
            while (added) {
                added = false;
                for (int i = remaining.size() - 1; i >= 0; i--) {
                    Pose3d ball = remaining.get(i);
                    boolean isNearCluster = poses.stream()
                            .anyMatch(inCluster ->
                                    distanceFrom(inCluster.toPose2d(), ball.toPose2d()) <= kFuelClusterGapMeters);

                    if (isNearCluster) {
                        poses.add(remaining.remove(i));
                        added = true;
                    }
                }
            }

            clusters.add(new FuelCluster(
                    poses.size(), distanceFrom(robotPose, poses.get(0).toPose2d()), poses));
        }

        return clusters;
    }

    private FuelCluster selectBestCluster(List<FuelCluster> clusters) {
        return clusters.stream()
                .max(Comparator.comparingInt(FuelCluster::fuelAmount)
                        .thenComparing(Comparator.comparingDouble(FuelCluster::distance)
                                .reversed()))
                .orElse(clusters.get(0));
    }

    // --- Manual Command Scheduler Logic ---
    private void startAction(Command command) {
        cancelAction(true);
        action = command;
        action.initialize();
        actionRunning = true;
    }

    private void runAction() {
        if (!actionRunning || action == null) return;

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
