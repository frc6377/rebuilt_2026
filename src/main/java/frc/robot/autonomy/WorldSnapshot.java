package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionIO.FuelObservation;
import java.util.List;

/** Immutable autonomy facts used by the objective planner. */
public record WorldSnapshot(
        boolean enabled,
        boolean redAlliance,
        boolean poseConfidenceHigh,
        boolean hasPossession,
        boolean hubActive,
        boolean scoringAllowed,
        Pose2d robotPose,
        double matchTime,
        int tagCount,
        int lastAcceptedPoseTagCount,
        double secondsSinceAcceptedPose,
        List<FuelObservation> fuelObservations) {}
