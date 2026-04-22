package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionConstants;
import gg.questnav.questnav.*;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

public class QuestNavIOReal implements QuestNavIO {
    private final @NotNull QuestNav questNav;
    private Pose3d questPose = new Pose3d();

    public QuestNavIOReal() {

        this.questNav = new QuestNav();
        this.questPose = new Pose3d();
        this.questNav.setPose(this.questPose);
    }

    /**
     * Resets the QuestNav pose to the specified robot pose.
     *
     * @param robotPose The robot pose to set.
     */
    public void resetQuestNavPose(@NotNull Pose3d robotPose) {
        Pose3d newQuestPose = robotPose.transformBy(VisionConstants.QuestNavConstants.ROBOT_TO_QUEST);
        this.questNav.setPose(newQuestPose);
    }

    public void zeroQuestNav() {
        this.questNav.setPose(
                new Pose3d(this.questPose.getTranslation(), VisionConstants.QuestNavConstants.ROBOT_TO_QUEST.getRotation()));
    }

    public void periodic() {
        this.questNav.commandPeriodic();
        PoseFrame[] questFrames = this.questNav.getAllUnreadPoseFrames();
        for (PoseFrame questFrame : questFrames) {
            if (questFrame.isTracking()) {
                this.questPose = questFrame.questPose3d();
                double timestamp = questFrame.dataTimestamp();
                Pose3d robotPose = this.questPose.transformBy(VisionConstants.QuestNavConstants.ROBOT_TO_QUEST.inverse());
                // consumer.accept(robotPose.toPose2d(), timestamp, VisionConstants.QUESTNAV_STD_DEVS);
            }
        }
    }

    /**
     * Sets the starting pose for QuestNav using the provided robot pose.
     *
     * @param pose The starting robot pose as a Pose3d.
     */
    public void setQuestNavStartPose(Pose3d pose) {
        this.questNav.setPose(pose);
    }

    /**
     * Returns a supplier for the current robot pose from QuestNav as a Pose3d.
     *
     * @return A supplier that provides the current robot pose in 3D field coordinates.
     */
    public @NotNull Supplier<Pose2d> getQuestNavPoseSupplier() {
        return () -> this.questPose
                .transformBy(VisionConstants.QuestNavConstants.ROBOT_TO_QUEST.inverse())
                .toPose2d();
    }
}
