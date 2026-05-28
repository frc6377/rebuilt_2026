package frc.robot.autonomy;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Tunable parameters and field objectives for the autonomy-first robot brain. */
public class AutonomyConfig implements AutonomySettings {
    private final LoggedNetworkNumber maxPathVelocity =
            new LoggedNetworkNumber("Autonomy/Config/MaxPathVelocityMps", 2.4);
    private final LoggedNetworkNumber maxPathAcceleration =
            new LoggedNetworkNumber("Autonomy/Config/MaxPathAccelerationMpsSq", 2.0);
    private final LoggedNetworkNumber maxAngularVelocityDeg =
            new LoggedNetworkNumber("Autonomy/Config/MaxAngularVelocityDegPerSec", 360.0);
    private final LoggedNetworkNumber maxAngularAccelerationDeg =
            new LoggedNetworkNumber("Autonomy/Config/MaxAngularAccelerationDegPerSecSq", 540.0);
    private final LoggedNetworkNumber pathTimeoutSeconds =
            new LoggedNetworkNumber("Autonomy/Config/PathTimeoutSeconds", 5.0);
    private final LoggedNetworkNumber goalToleranceMeters =
            new LoggedNetworkNumber("Autonomy/Config/GoalToleranceMeters", 0.75);
    private final LoggedNetworkNumber intakeTimeoutSeconds =
            new LoggedNetworkNumber("Autonomy/Config/IntakeTimeoutSeconds", 1.75);
    private final LoggedNetworkNumber scorePrepareTimeoutSeconds =
            new LoggedNetworkNumber("Autonomy/Config/ScorePrepareTimeoutSeconds", 1.5);
    private final LoggedNetworkNumber scoreFeedSeconds =
            new LoggedNetworkNumber("Autonomy/Config/ScoreFeedSeconds", 1.1);
    private final LoggedNetworkNumber recoverSeconds = new LoggedNetworkNumber("Autonomy/Config/RecoverSeconds", 0.4);
    private final LoggedNetworkNumber relocalizeSeconds =
            new LoggedNetworkNumber("Autonomy/Config/RelocalizeSeconds", 0.6);
    private final LoggedNetworkNumber waitForHubSeconds =
            new LoggedNetworkNumber("Autonomy/Config/WaitForHubSeconds", 0.5);
    private final LoggedNetworkNumber maxAcceptedPoseAgeSeconds =
            new LoggedNetworkNumber("Autonomy/Config/MaxAcceptedPoseAgeSeconds", 1.0);
    private final LoggedNetworkNumber odometryGraceSeconds =
            new LoggedNetworkNumber("Autonomy/Config/OdometryGraceSeconds", 3.0);
    private final LoggedNetworkNumber minAcceptedPoseTags =
            new LoggedNetworkNumber("Autonomy/Config/MinAcceptedPoseTags", 2.0);
    private final LoggedNetworkNumber maxRecoveryAttempts =
            new LoggedNetworkNumber("Autonomy/Config/MaxRecoveryAttempts", 3.0);
    private final LoggedNetworkNumber scoreDistanceMeters =
            new LoggedNetworkNumber("Autonomy/Config/ScoreDistanceMeters", 2.6);
    private final LoggedNetworkNumber visibleFuelMaxAgeSeconds =
            new LoggedNetworkNumber("Autonomy/Config/VisibleFuelMaxAgeSeconds", 0.75);
    private final LoggedNetworkNumber minFuelConfidence =
            new LoggedNetworkNumber("Autonomy/Config/MinFuelConfidence", 0.55);
    private final LoggedNetworkNumber failedObjectiveCooldownSeconds =
            new LoggedNetworkNumber("Autonomy/Config/FailedObjectiveCooldownSeconds", 2.0);

    public PathConstraints pathConstraints() {
        return new PathConstraints(
                maxPathVelocity.get(),
                maxPathAcceleration.get(),
                Degrees.of(maxAngularVelocityDeg.get()).in(Radians),
                Degrees.of(maxAngularAccelerationDeg.get()).in(Radians));
    }

    /** Blue-origin known FUEL pickup targets. These are mirrored for red alliance by the planner. */
    public List<Pose2d> knownFuelTargetsBlue() {
        return List.of(
                poseFacingHub(FieldConstants.Depot.depotCenter.toTranslation2d()),
                poseFacingHub(new Translation2d(1.35, FieldConstants.fieldWidth / 2.0)),
                poseFacingHub(
                        new Translation2d(FieldConstants.LinesVertical.hubCenter, FieldConstants.fieldWidth * 0.2)),
                poseFacingHub(
                        new Translation2d(FieldConstants.LinesVertical.hubCenter, FieldConstants.fieldWidth * 0.8)));
    }

    /** Blue-origin scoring poses near the blue hub. These are mirrored for red alliance by the planner. */
    public List<Pose2d> scoringPosesBlue() {
        double hubX = FieldConstants.Hub.topCenterPoint.getX();
        double hubY = FieldConstants.Hub.topCenterPoint.getY();
        double distance = scoreDistanceMeters.get();
        return List.of(
                poseFacingHub(new Translation2d(hubX - distance, hubY)),
                poseFacingHub(new Translation2d(hubX - distance * 0.85, hubY - 0.8)),
                poseFacingHub(new Translation2d(hubX - distance * 0.85, hubY + 0.8)));
    }

    private Pose2d poseFacingHub(Translation2d translation) {
        Translation2d hub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
        return new Pose2d(translation, hub.minus(translation).getAngle());
    }

    public Pose2d toCurrentAlliancePose(Pose2d bluePose, boolean redAlliance) {
        return redAlliance ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
    }

    public double pathTimeoutSeconds() {
        return Math.max(0.25, pathTimeoutSeconds.get());
    }

    public double goalToleranceMeters() {
        return Math.max(0.05, goalToleranceMeters.get());
    }

    public double intakeTimeoutSeconds() {
        return Math.max(0.1, intakeTimeoutSeconds.get());
    }

    public double scorePrepareTimeoutSeconds() {
        return Math.max(0.1, scorePrepareTimeoutSeconds.get());
    }

    public double scoreFeedSeconds() {
        return Math.max(0.1, scoreFeedSeconds.get());
    }

    public double recoverSeconds() {
        return Math.max(0.05, recoverSeconds.get());
    }

    public double relocalizeSeconds() {
        return Math.max(0.05, relocalizeSeconds.get());
    }

    public double waitForHubSeconds() {
        return Math.max(0.05, waitForHubSeconds.get());
    }

    public double maxAcceptedPoseAgeSeconds() {
        return Math.max(0.05, maxAcceptedPoseAgeSeconds.get());
    }

    public double odometryGraceSeconds() {
        return Math.max(0.0, odometryGraceSeconds.get());
    }

    public int minAcceptedPoseTags() {
        return Math.max(1, (int) Math.round(minAcceptedPoseTags.get()));
    }

    public int maxRecoveryAttempts() {
        return Math.max(0, (int) Math.round(maxRecoveryAttempts.get()));
    }

    public double visibleFuelMaxAgeSeconds() {
        return Math.max(0.05, visibleFuelMaxAgeSeconds.get());
    }

    public double minFuelConfidence() {
        return Math.max(0.0, Math.min(1.0, minFuelConfidence.get()));
    }

    public double failedObjectiveCooldownSeconds() {
        return Math.max(0.0, failedObjectiveCooldownSeconds.get());
    }
}
