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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.HubTagObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.constants.VisionConstants;
import frc.robot.subsystems.vision.questnav.QuestNavIO;
import frc.robot.util.NerfModeController;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    private final QuestNavIO questNavIO;
    private final NerfModeController nerfModeController;

    // QuestNav fields
    private final QuestNav questNav;
    private Pose3d questPose = new Pose3d();

    private final String[] logKeyInputs;
    private final String[] logKeyTagPoses;
    private final String[] logKeyRobotPoses;
    private final String[] logKeyRobotPosesAccepted;
    private final String[] logKeyRobotPosesRejected;

    private final Debouncer hasTagDebounce = new Debouncer(0.1, DebounceType.kFalling);

    public Vision(
            VisionConsumer consumer, QuestNavIO questNavIO, NerfModeController nerfModeController, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;
        this.questNavIO = questNavIO;
        this.nerfModeController = nerfModeController;

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

        if (Constants.EnabledSubsystems.kQuestNav) {

            // Initialize QuestNav
            questNav = new QuestNav();
            questPose = new Pose3d();
            questNav.setPose(questPose);
        } else {
            questNav = new QuestNav();
        }
    }

    // ========== Hub Distance / Facing ==========

    // All AprilTag IDs on the blue hub (two tags per face × 4 faces: 18/27, 19/20, 21/24, 25/26)
    private static final Set<Integer> BLUE_HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
    // All AprilTag IDs on the red hub (mirrors of blue: 5/8, 9/10, 11/2, 3/4)
    private static final Set<Integer> RED_HUB_TAG_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

    // Per-face middle (center) tag IDs — one representative tag per hub face.
    //   Blue: near=26, far=19, right=27, left=21
    //   Red:  near= 3, far=10, right= 5, left=11
    private static final Set<Integer> BLUE_HUB_MIDDLE_TAG_IDS = Set.of(26, 19, 27, 21);
    private static final Set<Integer> RED_HUB_MIDDLE_TAG_IDS = Set.of(3, 10, 5, 11);

    /**
     * Returns the distance from the camera to the closest visible alliance hub AprilTag.
     *
     * <p>Prefers a per-face middle tag ({@code BLUE/RED_HUB_MIDDLE_TAG_IDS}) for accuracy. Falls back to any visible
     * hub tag when no middle tag is in view. The raw {@code distToCamera} value reported by the Limelight rawfiducials
     * entry is returned directly.
     *
     * <p>Returns {@link OptionalDouble#empty()} when no hub tags are visible.
     */
    public OptionalDouble getHubDistance() {
        boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        Set<Integer> hubTagIds = isRed ? RED_HUB_TAG_IDS : BLUE_HUB_TAG_IDS;
        Set<Integer> middleTagIds = isRed ? RED_HUB_MIDDLE_TAG_IDS : BLUE_HUB_MIDDLE_TAG_IDS;
        if (inputs.length == 0 || inputs[0].hubTagObservations.length == 0) return OptionalDouble.empty();
        // Find the closest visible middle tag, falling back to any hub tag
        HubTagObservation bestMiddle = null;
        HubTagObservation bestAny = null;
        for (HubTagObservation obs : inputs[0].hubTagObservations) {
            if (middleTagIds.contains(obs.tagId())) {
                if (bestMiddle == null || obs.distToCamera() < bestMiddle.distToCamera()) {
                    bestMiddle = obs;
                }
            } else if (hubTagIds.contains(obs.tagId())) {
                if (bestAny == null || obs.distToCamera() < bestAny.distToCamera()) {
                    bestAny = obs;
                }
            }
        }
        Logger.recordOutput("Vision/HubDistance/BestMiddle", bestMiddle);
        Logger.recordOutput("Vision/HubDistance/BestAny", bestAny);
        HubTagObservation best = bestMiddle != null ? bestMiddle : bestAny;
        if (best == null) return OptionalDouble.empty();

        Logger.recordOutput("Vision/HubDistance/TagId", best.tagId());
        Logger.recordOutput("Vision/HubDistance/IsMiddleTag", bestMiddle != null);
        Logger.recordOutput("Vision/HubDistance/DistToCamera", best.distToCamera());
        Logger.recordOutput("Vision/HubDistance/DistToRobot", best.distToRobot());
        return OptionalDouble.of(best.distToCamera());
    }

    /**
     * Convenience wrapper — returns hub distance as a typed {@link Distance}, or {@code Meters.of(-1)} when
     * unavailable.
     */
    public Distance getHubDistanceMeasure() {
        return Meters.of(getHubDistance().orElse(-1.0));
    }

    /**
     * Returns true when the hub angle fallback has at least two hub tags visible and can produce a distance estimate.
     */
    public boolean hasHubDistanceFallback() {
        return getHubDistance().isPresent();
    }

    /**
     * Returns the direct Limelight-measured distance from the robot to the closest visible hub tag (meters), using the
     * {@code distToRobot} value from rawfiducials (stride-7 index 5).
     *
     * <p>Returns empty when no hub tags are visible.
     */
    public OptionalDouble getClosestHubTagDistance() {
        boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        Set<Integer> hubTagIds = isRed ? RED_HUB_TAG_IDS : BLUE_HUB_TAG_IDS;

        HubTagObservation best = null;
        for (VisionIOInputsAutoLogged inp : inputs) {
            for (HubTagObservation obs : inp.hubTagObservations) {
                if (hubTagIds.contains(obs.tagId())) {
                    if (best == null || obs.distToCamera() < best.distToCamera()) {
                        best = obs;
                    }
                }
            }
        }
        if (best == null) return OptionalDouble.empty();

        Logger.recordOutput("Vision/ClosestHubTag/TagId", best.tagId());
        Logger.recordOutput("Vision/ClosestHubTag/DistToCamera", best.distToCamera());
        Logger.recordOutput("Vision/ClosestHubTag/DistToRobot", best.distToRobot());
        return OptionalDouble.of(best.distToRobot());
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

    public void resetQuestNavPose(Pose3d robotPose) {
        questNavIO.resetQuestNavPose(robotPose);
    }

    public void zeroQuestNav() {
        questNavIO.zeroQuestNav();
    }

    public void setQuestNavStartPose(Pose3d pose) {
        questNavIO.setQuestNavStartPose(pose);
    }

    public Supplier<Pose2d> getQuestNavPoseSupplier() {
        return questNavIO.getQuestNavPoseSupplier();
    }
    /**
     * Gets the starting pose from the Limelight (camera index 0).
     *
     * @return The robot pose as a Pose3d, or null if no valid pose is available.
     */
    public Pose3d getStartingPoseFromLimelight() {
        return getStartingPoseFromCamera(0);
    }

    /**
     * Gets the tag count from the most recent observation of the specified camera.
     *
     * @param cameraIndex The index of the camera to use.
     * @return The number of tags seen, or 0 if no observations are available.
     */
    public int getTagCount(int cameraIndex) {
        if (cameraIndex >= inputs.length || !inputs[cameraIndex].connected) {
            return 0;
        }
        var observations = inputs[cameraIndex].poseObservations;
        if (observations.length == 0) {
            return 0;
        }
        var latestObservation = observations[observations.length - 1];
        return latestObservation.tagCount();
    }

    public int getTagCount() {
        int tagCount = 0;
        for (var camera : inputs) {
            tagCount += camera.tagIds.length;
        }
        return tagCount;
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
                var tagPose =
                        nerfModeController.getVisionConstants().aprilTagLayout().getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                VisionConstants constants = nerfModeController.getVisionConstants();
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() < constants.minCameras() // Must have at least one tag
                        || observation.ambiguity() > constants.maxAmbiguity() // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ())
                                > constants.maxZError() // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX()
                                > constants.aprilTagLayout().getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY()
                                > constants.aprilTagLayout().getFieldWidth();

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
                double linearStdDev = constants.linearStdDevBaseline() * stdDevFactor;
                double angularStdDev;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    // MegaTag2 rotation is seeded from the gyro — trust it normally
                    linearStdDev *= constants.linearStdDevMegatag2Factor();
                    angularStdDev =
                            constants.angularStdDevBaseline() * stdDevFactor * constants.angularStdDevMegatag2Factor();
                } else {
                    // MegaTag1 rotation is computed from vision alone — never let it influence
                    // the pose estimator's heading (field-oriented drive would snap on every tag)
                    angularStdDev = Double.MAX_VALUE;
                }
                if (cameraIndex < constants.cameraStdDevFactors().length) {
                    linearStdDev *= constants.cameraStdDevFactors()[cameraIndex];
                    // Do not scale angular if it is already pinned to MAX_VALUE
                    if (angularStdDev < Double.MAX_VALUE) {
                        angularStdDev *= constants.cameraStdDevFactors()[cameraIndex];
                    }
                }

                // Send vision observation
                if (!DriverStation.isAutonomousEnabled()) {
                    consumer.accept(
                            observation.pose().toPose2d(),
                            observation.timestamp(),
                            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                }
            }

            // Log camera data
            Logger.recordOutput("Vision/HasTag", hasTagDebounce.calculate(!tagPoses.isEmpty()));
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

        if (Constants.EnabledSubsystems.kQuestNav) {
            // QuestNav periodic
            questNav.commandPeriodic();
            PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
            for (PoseFrame questFrame : questFrames) {
                if (questFrame.isTracking()) {
                    questPose = questFrame.questPose3d();
                    double timestamp = questFrame.dataTimestamp();
                    Pose3d robotPose = questPose.transformBy(nerfModeController
                            .getVisionConstants()
                            .robotToQuest()
                            .inverse());
                    consumer.accept(
                            robotPose.toPose2d(),
                            timestamp,
                            nerfModeController.getVisionConstants().questNavStdDevs());
                }
            }
            Logger.recordOutput("Vision/QuestNav/Connected", questNav.isConnected());
            Logger.recordOutput("Vision/QuestNav/Pose", questPose);
        }
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
