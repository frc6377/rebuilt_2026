package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionIO.FuelObservation;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/** Scores possible robot objectives and chooses the highest-value valid task. */
public class ObjectivePlanner {
    private final Map<String, Double> blockedUntil = new HashMap<>();
    private final DoubleSupplier timestampSupplier;
    private final ObjectiveScoringModel scoringModel;

    public ObjectivePlanner() {
        this(Timer::getFPGATimestamp);
    }

    public ObjectivePlanner(DoubleSupplier timestampSupplier) {
        this(timestampSupplier, new DeterministicObjectiveScoringModel());
    }

    public ObjectivePlanner(DoubleSupplier timestampSupplier, ObjectiveScoringModel scoringModel) {
        this.timestampSupplier = timestampSupplier;
        this.scoringModel = Objects.requireNonNull(scoringModel);
    }

    public void reset() {
        blockedUntil.clear();
    }

    public String scoringModelName() {
        return scoringModel.name();
    }

    public void markFailed(AutonomyObjective objective, AutonomySettings config) {
        if (objective == null
                || objective.targetId() == null
                || objective.targetId().isBlank()) {
            return;
        }
        blockedUntil.put(
                objective.targetId(), timestampSupplier.getAsDouble() + config.failedObjectiveCooldownSeconds());
    }

    public AutonomyObjective select(WorldSnapshot world, AutonomySettings config) {
        if (!world.enabled()) {
            return AutonomyObjective.stop("Disabled");
        }
        if (!world.poseConfidenceHigh()) {
            return new AutonomyObjective(AutonomyObjectiveType.RELOCALIZE, world.robotPose(), "Relocalize", 1.0, false);
        }
        if (world.hasPossession()) {
            if (!world.hubActive()) {
                return new AutonomyObjective(
                        AutonomyObjectiveType.WAIT_FOR_HUB, world.robotPose(), "WaitForHub", 2.0, false);
            }
            return bestScoringObjective(world, config);
        }

        AutonomyObjective fuelObjective =
                higherScore(bestVisibleFuelObjective(world, config), bestKnownFuelObjective(world, config));
        if (fuelObjective != null) {
            return fuelObjective;
        }

        return new AutonomyObjective(AutonomyObjectiveType.RECOVER, world.robotPose(), "NoFuelTarget", 0.5, false);
    }

    private AutonomyObjective bestVisibleFuelObjective(WorldSnapshot world, AutonomySettings config) {
        AutonomyObjective best = null;
        double now = timestampSupplier.getAsDouble();
        for (FuelObservation observation : world.fuelObservations()) {
            if (!observation.hasFieldPose() || observation.confidence() < config.minFuelConfidence()) {
                continue;
            }
            double ageSeconds = Math.max(0.0, now - observation.timestampSeconds());
            if (ageSeconds > config.visibleFuelMaxAgeSeconds() || isBlocked(observation.source())) {
                continue;
            }
            double distance = world.robotPose()
                    .getTranslation()
                    .getDistance(observation.fieldPose().getTranslation());
            double score = scoringModel.scoreVisibleFuel(observation, distance, ageSeconds, world);
            if (best == null || score > best.score()) {
                best = new AutonomyObjective(
                        AutonomyObjectiveType.COLLECT_VISIBLE_FUEL,
                        observation.fieldPose(),
                        observation.source(),
                        score,
                        true);
            }
        }
        return best;
    }

    private AutonomyObjective bestKnownFuelObjective(WorldSnapshot world, AutonomySettings config) {
        AutonomyObjective best = null;
        int index = 0;
        for (Pose2d bluePose : config.knownFuelTargetsBlue()) {
            Pose2d pose = config.toCurrentAlliancePose(bluePose, world.redAlliance());
            String targetId = "KnownFuel:" + index++;
            if (isBlocked(targetId)) {
                continue;
            }
            double distance = world.robotPose().getTranslation().getDistance(pose.getTranslation());
            double score = scoringModel.scoreKnownFuel(pose, distance, world);
            if (best == null || score > best.score()) {
                best = new AutonomyObjective(AutonomyObjectiveType.COLLECT_KNOWN_FUEL, pose, targetId, score, false);
            }
        }
        return best;
    }

    private AutonomyObjective bestScoringObjective(WorldSnapshot world, AutonomySettings config) {
        AutonomyObjective best = null;
        int index = 0;
        for (Pose2d bluePose : config.scoringPosesBlue()) {
            Pose2d pose = config.toCurrentAlliancePose(bluePose, world.redAlliance());
            String targetId = "ScoreHub:" + index++;
            if (isBlocked(targetId)) {
                continue;
            }
            double distance = world.robotPose().getTranslation().getDistance(pose.getTranslation());
            double score = scoringModel.scoreScoringPose(pose, distance, world);
            if (best == null || score > best.score()) {
                best = new AutonomyObjective(AutonomyObjectiveType.SCORE_HUB, pose, targetId, score, false);
            }
        }
        if (best == null) {
            return new AutonomyObjective(AutonomyObjectiveType.RECOVER, world.robotPose(), "NoScoreTarget", 0.5, false);
        }
        return best;
    }

    private boolean isBlocked(String targetId) {
        Double blockedTimestamp = blockedUntil.get(targetId);
        if (blockedTimestamp == null) {
            return false;
        }
        if (timestampSupplier.getAsDouble() >= blockedTimestamp) {
            blockedUntil.remove(targetId);
            return false;
        }
        return true;
    }

    private static AutonomyObjective higherScore(AutonomyObjective first, AutonomyObjective second) {
        if (first == null) {
            return second;
        }
        if (second == null) {
            return first;
        }
        return first.score() >= second.score() ? first : second;
    }
}
