package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionIO.FuelObservation;
import java.util.List;
import org.junit.jupiter.api.Test;

class ObjectivePlannerTest {
    private static final double NOW_SECONDS = 42.0;
    private static final TestSettings CONFIG = new TestSettings();

    @Test
    void disabledRobotStops() {
        assertEquals(
                AutonomyObjectiveType.STOP,
                planner()
                        .select(snapshot(false, true, false, true, true, List.of()), CONFIG)
                        .type());
    }

    @Test
    void lowPoseConfidenceRelocalizes() {
        assertEquals(
                AutonomyObjectiveType.RELOCALIZE,
                planner()
                        .select(snapshot(true, false, false, true, true, List.of()), CONFIG)
                        .type());
    }

    @Test
    void validVisibleFuelBeatsKnownTarget() {
        FuelObservation observation = new FuelObservation(
                NOW_SECONDS, new Pose2d(2.0, 0.0, new Rotation2d()), true, new Rotation2d(), 2.0, 0.9, "VisionFuel");

        assertEquals(
                AutonomyObjectiveType.COLLECT_VISIBLE_FUEL,
                planner()
                        .select(snapshot(true, true, false, true, true, List.of(observation)), CONFIG)
                        .type());
    }

    @Test
    void injectedScoringModelCanReprioritizeValidFuelCandidates() {
        FuelObservation observation = new FuelObservation(
                NOW_SECONDS, new Pose2d(2.0, 0.0, new Rotation2d()), true, new Rotation2d(), 2.0, 0.9, "VisionFuel");
        ObjectiveScoringModel knownFuelAdvisor = new ObjectiveScoringModel() {
            @Override
            public String name() {
                return "TestKnownFuelAdvisor";
            }

            @Override
            public double scoreVisibleFuel(
                    FuelObservation observation, double distanceMeters, double ageSeconds, WorldSnapshot world) {
                return -1.0;
            }

            @Override
            public double scoreKnownFuel(Pose2d targetPose, double distanceMeters, WorldSnapshot world) {
                return 100.0 - distanceMeters;
            }

            @Override
            public double scoreScoringPose(Pose2d targetPose, double distanceMeters, WorldSnapshot world) {
                return 0.0;
            }
        };

        assertEquals(
                AutonomyObjectiveType.COLLECT_KNOWN_FUEL,
                new ObjectivePlanner(() -> NOW_SECONDS, knownFuelAdvisor)
                        .select(snapshot(true, true, false, true, true, List.of(observation)), CONFIG)
                        .type());
    }

    @Test
    void staleVisibleFuelFallsBackToKnownTarget() {
        FuelObservation observation = new FuelObservation(
                NOW_SECONDS - 10.0,
                new Pose2d(2.0, 0.0, new Rotation2d()),
                true,
                new Rotation2d(),
                2.0,
                0.9,
                "StaleFuel");

        assertEquals(
                AutonomyObjectiveType.COLLECT_KNOWN_FUEL,
                planner()
                        .select(snapshot(true, true, false, true, true, List.of(observation)), CONFIG)
                        .type());
    }

    @Test
    void possessionWithInactiveHubWaits() {
        assertEquals(
                AutonomyObjectiveType.WAIT_FOR_HUB,
                planner()
                        .select(snapshot(true, true, true, false, false, List.of()), CONFIG)
                        .type());
    }

    @Test
    void possessionWithActiveHubScores() {
        assertEquals(
                AutonomyObjectiveType.SCORE_HUB,
                planner()
                        .select(snapshot(true, true, true, true, true, List.of()), CONFIG)
                        .type());
    }

    @Test
    void failedVisibleTargetIsTemporarilyIgnored() {
        ObjectivePlanner planner = planner();
        FuelObservation observation = new FuelObservation(
                NOW_SECONDS, new Pose2d(2.0, 0.0, new Rotation2d()), true, new Rotation2d(), 2.0, 0.9, "VisionFuel");
        WorldSnapshot snapshot = snapshot(true, true, false, true, true, List.of(observation));

        AutonomyObjective first = planner.select(snapshot, CONFIG);
        planner.markFailed(first, CONFIG);

        assertEquals(
                AutonomyObjectiveType.COLLECT_KNOWN_FUEL,
                planner.select(snapshot, CONFIG).type());
    }

    private static ObjectivePlanner planner() {
        return new ObjectivePlanner(() -> NOW_SECONDS);
    }

    private static WorldSnapshot snapshot(
            boolean enabled,
            boolean poseConfidenceHigh,
            boolean hasPossession,
            boolean hubActive,
            boolean scoringAllowed,
            List<FuelObservation> fuelObservations) {
        return new WorldSnapshot(
                enabled,
                false,
                poseConfidenceHigh,
                hasPossession,
                hubActive,
                scoringAllowed,
                new Pose2d(),
                100.0,
                2,
                2,
                0.1,
                fuelObservations);
    }

    private static class TestSettings implements AutonomySettings {
        @Override
        public List<Pose2d> knownFuelTargetsBlue() {
            return List.of(new Pose2d(4.0, 0.0, new Rotation2d()));
        }

        @Override
        public List<Pose2d> scoringPosesBlue() {
            return List.of(new Pose2d(1.0, 0.0, new Rotation2d()));
        }

        @Override
        public Pose2d toCurrentAlliancePose(Pose2d bluePose, boolean redAlliance) {
            return bluePose;
        }

        @Override
        public double visibleFuelMaxAgeSeconds() {
            return 0.75;
        }

        @Override
        public double minFuelConfidence() {
            return 0.55;
        }

        @Override
        public double failedObjectiveCooldownSeconds() {
            return 2.0;
        }
    }
}
