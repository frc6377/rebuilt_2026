// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.HubTagObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    private final String[] logKeyInputs;
    private final String[] logKeyTagPoses;
    private final String[] logKeyRobotPoses;
    private final String[] logKeyRobotPosesAccepted;
    private final String[] logKeyRobotPosesRejected;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs and log keys
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        this.logKeyInputs = new String[io.length];
        this.logKeyTagPoses = new String[io.length];
        this.logKeyRobotPoses = new String[io.length];
        this.logKeyRobotPosesAccepted = new String[io.length];
        this.logKeyRobotPosesRejected = new String[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
            String cameraPrefix = "Vision/Camera" + i;
            logKeyInputs[i] = cameraPrefix;
            logKeyTagPoses[i] = cameraPrefix + "/TagPoses";
            logKeyRobotPoses[i] = cameraPrefix + "/RobotPoses";
            logKeyRobotPosesAccepted[i] = cameraPrefix + "/RobotPosesAccepted";
            logKeyRobotPosesRejected[i] = cameraPrefix + "/RobotPosesRejected";
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < io.length; i++) {
            disconnectedAlerts[i] = new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
        }
    }

    // ========== Hub Distance Fallback ==========

    // All AprilTag IDs on the blue hub (two tags per face × 4 faces: 18/27, 19/20, 21/24, 25/26)
    private static final Set<Integer> BLUE_HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
    // All AprilTag IDs on the red hub (mirrors of blue: 5/8, 9/10, 11/2, 3/4)
    private static final Set<Integer> RED_HUB_TAG_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

    // Same-face tag pairs — tags on the same physical face of the hub.
    // These give the most geometrically stable distance reading because their
    // known separation is small and well-defined.
    //   Blue:  face pairs (18,27), (19,20), (21,24), (25,26)
    //   Red:   face pairs ( 5, 8), ( 9,10), ( 2,11), ( 3, 4)
    private static final int[][] BLUE_HUB_FACE_PAIRS = {{18, 27}, {19, 20}, {21, 24}, {25, 26}};
    private static final int[][] RED_HUB_FACE_PAIRS  = {{ 5,  8}, { 9, 10}, { 2, 11}, { 3,  4}};

    /**
     * Returns the distance from the camera to the alliance hub, estimated purely from the angular
     * separation θ between any two simultaneously-visible hub AprilTags.
     *
     * <p>Formula: {@code D = S / (2 * tan(θ / 2))} where S is the known 3D distance between the
     * two tag poses in the field layout.
     *
     * <p>Same-face tag pairs (e.g. 25 &amp; 26, which share a hub face) are tried first because
     * their fixed separation is small and well-defined, giving the most accurate reading. Any other
     * visible hub-tag pair is used as a secondary fallback.
     *
     * <p>Returns {@link OptionalDouble#empty()} when fewer than two hub tags are visible.
     */
    public OptionalDouble getHubDistance() {
        boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red;
        Set<Integer> hubTagIds = isRed ? RED_HUB_TAG_IDS : BLUE_HUB_TAG_IDS;
        int[][] facePairs = isRed ? RED_HUB_FACE_PAIRS : BLUE_HUB_FACE_PAIRS;

        // Collect all visible hub-tag observations across every camera, keyed by tag ID
        Map<Integer, Rotation2d> visibleTags = new HashMap<>();
        for (VisionIOInputsAutoLogged inp : inputs) {
            for (HubTagObservation obs : inp.hubTagObservations) {
                if (hubTagIds.contains(obs.tagId())) {
                    visibleTags.put(obs.tagId(), obs.tx());
                }
            }
        }

        if (visibleTags.size() < 2) {
            return OptionalDouble.empty();
        }

        // Try same-face pairs first (most accurate), then all other pairs as fallback
        List<int[]> candidates = new ArrayList<>();
        for (int[] pair : facePairs) {
            if (visibleTags.containsKey(pair[0]) && visibleTags.containsKey(pair[1])) {
                candidates.add(0, pair); // prepend — try these first
            }
        }
        // Append all remaining cross-face combinations
        List<Integer> ids = new ArrayList<>(visibleTags.keySet());
        for (int a = 0; a < ids.size(); a++) {
            for (int b = a + 1; b < ids.size(); b++) {
                int[] pair = {ids.get(a), ids.get(b)};
                // Only add if not already a same-face pair already in candidates
                boolean alreadyAdded = false;
                for (int[] c : candidates) {
                    if ((c[0] == pair[0] && c[1] == pair[1]) || (c[0] == pair[1] && c[1] == pair[0])) {
                        alreadyAdded = true;
                        break;
                    }
                }
                if (!alreadyAdded) candidates.add(pair);
            }
        }

        for (int[] pair : candidates) {
            int idA = pair[0], idB = pair[1];
            var poseA = aprilTagLayout.getTagPose(idA);
            var poseB = aprilTagLayout.getTagPose(idB);
            if (poseA.isEmpty() || poseB.isEmpty()) continue;

            double separation = poseA.get().getTranslation().getDistance(poseB.get().getTranslation());
            if (separation < 0.01) continue;

            double thetaRad = Math.abs(
                    visibleTags.get(idA).getRadians() - visibleTags.get(idB).getRadians());
            if (thetaRad < 1e-6) continue;

            double distance = separation / (2.0 * Math.tan(thetaRad / 2.0));
            Logger.recordOutput("Vision/HubAngleFallback/TagA", idA);
            Logger.recordOutput("Vision/HubAngleFallback/TagB", idB);
            Logger.recordOutput("Vision/HubAngleFallback/ThetaDeg", Math.toDegrees(thetaRad));
            Logger.recordOutput("Vision/HubAngleFallback/SeparationM", separation);
            Logger.recordOutput("Vision/HubAngleFallback/DistanceM", distance);
            return OptionalDouble.of(distance);
        }
        return OptionalDouble.empty();
    }

    /**
     * Convenience wrapper — returns hub distance as a typed {@link Distance}, or
     * {@code Meters.of(-1)} when unavailable.
     */
    public Distance getHubDistanceMeasure() {
        return Meters.of(getHubDistance().orElse(-1.0));
    }

    /**
     * Returns true when the hub angle fallback has at least two hub tags visible and can produce a
     * distance estimate.
     */
    public boolean hasHubDistanceFallback() {
        return getHubDistance().isPresent();
    }

    // ========== Standard Vision ==========

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    public int getTagCount(int cameraIndex) {
        var observations = inputs[cameraIndex].poseObservations;
        if (observations.length == 0) {
            return 0;
        }
        var latestObservation = observations[observations.length - 1];
        return latestObservation.tagCount();
    }

    public Pose3d getStartingPoseFromCamera(int cameraIndex) {
        if (cameraIndex >= inputs.length || !inputs[cameraIndex].connected) {
            return null;
        }

        var observations = inputs[cameraIndex].poseObservations;
        if (observations.length == 0) {
            return null;
        }

        // Return the most recent pose observation with at least one tag
        var latestObservation = observations[observations.length - 1];
        if (latestObservation.tagCount() > 0) {
            return latestObservation.pose();
        }
        return null;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs(logKeyInputs[i], inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera data
            Logger.recordOutput(logKeyTagPoses[cameraIndex], tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(logKeyRobotPoses[cameraIndex], robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    logKeyRobotPosesAccepted[cameraIndex],
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    logKeyRobotPosesRejected[cameraIndex],
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
