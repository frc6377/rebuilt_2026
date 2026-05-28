package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionIO.FuelObservation;

/** Default hand-tuned scoring model used until an AI/ML advisor is explicitly added. */
public class DeterministicObjectiveScoringModel implements ObjectiveScoringModel {
    @Override
    public String name() {
        return "DeterministicV1";
    }

    @Override
    public double scoreVisibleFuel(
            FuelObservation observation, double distanceMeters, double ageSeconds, WorldSnapshot world) {
        return 10.0 + observation.confidence() * 3.0 - distanceMeters * 0.45 - ageSeconds;
    }

    @Override
    public double scoreKnownFuel(Pose2d targetPose, double distanceMeters, WorldSnapshot world) {
        return 6.0 - distanceMeters * 0.35;
    }

    @Override
    public double scoreScoringPose(Pose2d targetPose, double distanceMeters, WorldSnapshot world) {
        return 9.0 - distanceMeters * 0.25 + (world.scoringAllowed() ? 3.0 : 0.0);
    }
}
