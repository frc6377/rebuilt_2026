package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;

/** A task-level objective chosen by the autonomy planner. */
public record AutonomyObjective(
        AutonomyObjectiveType type, Pose2d targetPose, String targetId, double score, boolean visionTarget) {
    public static AutonomyObjective stop(String reason) {
        return new AutonomyObjective(AutonomyObjectiveType.STOP, new Pose2d(), reason, 0.0, false);
    }
}
