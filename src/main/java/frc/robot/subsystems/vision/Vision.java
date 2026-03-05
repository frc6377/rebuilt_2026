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
import static frc.robot.subsystems.vision.VisionConstants.angularStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.angularStdDevMegatag2Factor;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.cameraStdDevFactors;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevMegatag2Factor;
import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;
import static frc.robot.subsystems.vision.VisionConstants.maxZError;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.HubTagObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.questnav.QuestNavIO;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
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

    // QuestNav fields
    private final QuestNav questNav;
    private Pose3d questPose = new Pose3d();

    private final String[] logKeyInputs;
    private final String[] logKeyTagPoses;
    private final String[] logKeyRobotPoses;
    private final String[] logKeyRobotPosesAccepted;
    private final String[] logKeyRobotPosesRejected;

    public Vision(VisionConsumer consumer, QuestNavIO questNavIO, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;
        this.questNavIO = questNavIO;

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

    /**
     * Returns the field-relative heading the robot must face to be perpendicular to the visible hub face.
     *
     * <p>Selects the closest visible middle hub tag (preferred) or any hub tag as a fallback. The required heading is
     * computed as the bearing from {@code robotPose} to the tag's field-layout position, using the robot's gyro-derived
     * rotation embedded in {@code robotPose}.
     *
     * @param robotPose Current robot pose (translation + gyro rotation) from the drive odometry.
     * @return Required heading to face the hub, or empty if no hub tags are visible.
     */
    public Optional<Rotation2d> getHubFacingAngle(Pose2d robotPose) {
        if (inputs.length == 0) return getHubFacingAngleGyroFallback(robotPose);
        boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        Set<Integer> hubTagIds = isRed ? RED_HUB_TAG_IDS : BLUE_HUB_TAG_IDS;
        Set<Integer> middleTagIds = isRed ? RED_HUB_MIDDLE_TAG_IDS : BLUE_HUB_MIDDLE_TAG_IDS;

        // Find the closest visible middle tag; fall back to any hub tag
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
        HubTagObservation best = bestMiddle != null ? bestMiddle : bestAny;
        if (best == null) return getHubFacingAngleGyroFallback(robotPose);

        // Resolve the tag's field-layout position
        var tagPoseOpt = aprilTagLayout.getTagPose(best.tagId());
        if (tagPoseOpt.isEmpty()) return getHubFacingAngleGyroFallback(robotPose);

        // Bearing from robot to the tag's field position (gyro heading is in robotPose.getRotation())
        double dx = tagPoseOpt.get().getX() - robotPose.getX();
        double dy = tagPoseOpt.get().getY() - robotPose.getY();
        // Point directly at the tag (front of robot faces the hub)
        Rotation2d required = new Rotation2d(dx, dy).rotateBy(Rotation2d.fromDegrees(180.0));
        Rotation2d error = required.minus(robotPose.getRotation());

        Logger.recordOutput("Vision/HubFacing/TagId", best.tagId());
        Logger.recordOutput("Vision/HubFacing/IsMiddleTag", bestMiddle != null);
        Logger.recordOutput("Vision/HubFacing/GyroFallback", false);
        Logger.recordOutput("Vision/HubFacing/TagFieldX", tagPoseOpt.get().getX());
        Logger.recordOutput("Vision/HubFacing/TagFieldY", tagPoseOpt.get().getY());
        Logger.recordOutput("Vision/HubFacing/RequiredHeadingDeg", required.getDegrees());
        Logger.recordOutput("Vision/HubFacing/HeadingErrorDeg", error.getDegrees());
        return Optional.of(required);
    }

    /**
     * Gyro-only fallback: bearing from the robot's odometry pose toward the alliance hub center. Used when no hub
     * AprilTags are visible (neutral zone, camera obstructed, etc.).
     */
    private Optional<Rotation2d> getHubFacingAngleGyroFallback(Pose2d robotPose) {
        boolean isRed = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        // Pick the center of our alliance hub
        Translation2d hubCenter = isRed
                ? new Translation2d(
                        FieldConstants.Hub.oppTopCenterPoint.getX(), FieldConstants.Hub.oppTopCenterPoint.getY())
                : new Translation2d(FieldConstants.Hub.topCenterPoint.getX(), FieldConstants.Hub.topCenterPoint.getY());

        double dx = hubCenter.getX() - robotPose.getX();
        double dy = hubCenter.getY() - robotPose.getY();
        Rotation2d required = new Rotation2d(dx, dy);
        Rotation2d error = required.minus(robotPose.getRotation());

        Logger.recordOutput("Vision/HubFacing/GyroFallback", true);
        Logger.recordOutput("Vision/HubFacing/RequiredHeadingDeg", required.getDegrees());
        Logger.recordOutput("Vision/HubFacing/HeadingErrorDeg", error.getDegrees());
        return Optional.of(required);
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
        for (var camera: inputs) {
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
                double angularStdDev;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    // MegaTag2 rotation is seeded from the gyro — trust it normally
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev = angularStdDevBaseline * stdDevFactor * angularStdDevMegatag2Factor;
                } else {
                    // MegaTag1 rotation is computed from vision alone — never let it influence
                    // the pose estimator's heading (field-oriented drive would snap on every tag)
                    angularStdDev = Double.MAX_VALUE;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    // Do not scale angular if it is already pinned to MAX_VALUE
                    if (angularStdDev < Double.MAX_VALUE) {
                        angularStdDev *= cameraStdDevFactors[cameraIndex];
                    }
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

        if (Constants.EnabledSubsystems.kQuestNav) {
            // QuestNav periodic
            questNav.commandPeriodic();
            PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
            for (PoseFrame questFrame : questFrames) {
                if (questFrame.isTracking()) {
                    questPose = questFrame.questPose3d();
                    double timestamp = questFrame.dataTimestamp();
                    Pose3d robotPose = questPose.transformBy(VisionConstants.ROBOT_TO_QUEST.inverse());
                    consumer.accept(robotPose.toPose2d(), timestamp, VisionConstants.QUESTNAV_STD_DEVS);
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
