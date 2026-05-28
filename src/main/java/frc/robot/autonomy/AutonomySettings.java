package frc.robot.autonomy;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

/** Planner-facing autonomy settings. */
public interface AutonomySettings {
    List<Pose2d> knownFuelTargetsBlue();

    List<Pose2d> scoringPosesBlue();

    Pose2d toCurrentAlliancePose(Pose2d bluePose, boolean redAlliance);

    double visibleFuelMaxAgeSeconds();

    double minFuelConfidence();

    double failedObjectiveCooldownSeconds();
}
