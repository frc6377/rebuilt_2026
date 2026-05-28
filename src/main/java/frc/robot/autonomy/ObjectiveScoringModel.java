package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionIO.FuelObservation;

/**
 * Scores planner candidates after deterministic validity checks have already passed.
 *
 * <p>Future AI/ML strategy modules can implement this interface to reprioritize objectives, while the planner keeps
 * hard safety gates such as pose confidence, stale-data rejection, and failed-target cooldowns in robot code.
 */
public interface ObjectiveScoringModel {
    String name();

    double scoreVisibleFuel(FuelObservation observation, double distanceMeters, double ageSeconds, WorldSnapshot world);

    double scoreKnownFuel(Pose2d targetPose, double distanceMeters, WorldSnapshot world);

    double scoreScoringPose(Pose2d targetPose, double distanceMeters, WorldSnapshot world);
}
