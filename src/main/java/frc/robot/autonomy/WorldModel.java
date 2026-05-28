package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.FuelObservation;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Fuses robot, field, mechanism, and perception facts for the autonomy planner. */
public class WorldModel {
    private double enabledStartTimestamp = Timer.getFPGATimestamp();
    private WorldSnapshot snapshot = new WorldSnapshot(
            false, false, false, false, false, false, new Pose2d(), 0.0, 0, 0, Double.POSITIVE_INFINITY, List.of());

    public void reset() {
        enabledStartTimestamp = Timer.getFPGATimestamp();
        snapshot = new WorldSnapshot(
                false, false, false, false, false, false, new Pose2d(), 0.0, 0, 0, Double.POSITIVE_INFINITY, List.of());
    }

    public void update(
            Drive drive,
            Vision vision,
            RobotState robotState,
            Superstructure superstructure,
            AutonomyConfig config,
            boolean timedPossessionFallback) {
        Pose2d robotPose = drive.getPose();
        int tagCount = vision.getTagCount();
        int lastAcceptedPoseTagCount = vision.getLastAcceptedPoseTagCount();
        double secondsSinceAcceptedPose = vision.getSecondsSinceLastAcceptedPose();
        boolean redAlliance =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        boolean recentAcceptedPose = lastAcceptedPoseTagCount >= config.minAcceptedPoseTags()
                && secondsSinceAcceptedPose <= config.maxAcceptedPoseAgeSeconds();
        boolean odometryGrace = Timer.getFPGATimestamp() - enabledStartTimestamp <= config.odometryGraceSeconds();
        boolean poseConfidenceHigh = recentAcceptedPose || odometryGrace || Constants.currentMode == Constants.Mode.SIM;

        boolean hasPossession = Constants.currentMode == Constants.Mode.SIM
                ? robotState.getSimGamePieceCount() > 0 || timedPossessionFallback
                : timedPossessionFallback;
        boolean hubActive = FieldConstants.isHubActive();
        boolean scoringAllowed = poseConfidenceHigh && hubActive && superstructure.isInShootingZone(robotPose);
        List<FuelObservation> fuelObservations = List.copyOf(vision.getFuelObservations());

        snapshot = new WorldSnapshot(
                DriverStation.isEnabled(),
                redAlliance,
                poseConfidenceHigh,
                hasPossession,
                hubActive,
                scoringAllowed,
                robotPose,
                DriverStation.getMatchTime(),
                tagCount,
                lastAcceptedPoseTagCount,
                secondsSinceAcceptedPose,
                fuelObservations);
    }

    public WorldSnapshot snapshot() {
        return snapshot;
    }

    public void log() {
        Logger.recordOutput("Autonomy/World/Enabled", snapshot.enabled());
        Logger.recordOutput("Autonomy/World/RedAlliance", snapshot.redAlliance());
        Logger.recordOutput("Autonomy/World/Pose", snapshot.robotPose());
        Logger.recordOutput("Autonomy/World/PoseConfidenceHigh", snapshot.poseConfidenceHigh());
        Logger.recordOutput("Autonomy/World/HasPossession", snapshot.hasPossession());
        Logger.recordOutput("Autonomy/World/HubActive", snapshot.hubActive());
        Logger.recordOutput("Autonomy/World/ScoringAllowed", snapshot.scoringAllowed());
        Logger.recordOutput("Autonomy/World/TagCount", snapshot.tagCount());
        Logger.recordOutput("Autonomy/World/LastAcceptedPoseTagCount", snapshot.lastAcceptedPoseTagCount());
        Logger.recordOutput("Autonomy/World/SecondsSinceAcceptedPose", snapshot.secondsSinceAcceptedPose());
        Logger.recordOutput(
                "Autonomy/World/FuelObservationCount",
                snapshot.fuelObservations().size());
        Logger.recordOutput(
                "Autonomy/World/FuelObservationSources",
                snapshot.fuelObservations().stream()
                        .map(FuelObservation::source)
                        .toArray(String[]::new));
        Logger.recordOutput(
                "Autonomy/World/FuelObservationConfidences",
                snapshot.fuelObservations().stream()
                        .mapToDouble(FuelObservation::confidence)
                        .toArray());
        Logger.recordOutput(
                "Autonomy/World/FuelObservationTimestamps",
                snapshot.fuelObservations().stream()
                        .mapToDouble(FuelObservation::timestampSeconds)
                        .toArray());
        Logger.recordOutput("Autonomy/World/MatchTime", snapshot.matchTime());
    }
}
