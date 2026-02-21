package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionConstants;
import gg.questnav.questnav.*;
import java.util.function.Supplier;

public class QuestNavIOReal implements QuestNavIO {
    private final QuestNav questNav;
    private Pose3d questPose = new Pose3d();

    public QuestNavIOReal() {

        questNav = new QuestNav();
        questPose = new Pose3d();
        questNav.setPose(questPose);
    }

    /**
     * Resets the QuestNav pose to the specified robot pose.
     *
     * @param robotPose The robot pose to set.
     */
    public void resetQuestNavPose(Pose3d robotPose) {
        Pose3d newQuestPose = robotPose.transformBy(VisionConstants.QuestNavConstants.ROBOT_TO_QUEST);
        questNav.setPose(newQuestPose);
    }

    public void zeroQuestNav() {
        questNav.setPose(
                new Pose3d(questPose.getTranslation(), VisionConstants.QuestNavConstants.ROBOT_TO_QUEST.getRotation()));
    }

    public void periodic() {
        questNav.commandPeriodic();
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
        for (PoseFrame questFrame : questFrames) {
            if (questFrame.isTracking()) {
                questPose = questFrame.questPose3d();
                double timestamp = questFrame.dataTimestamp();
                Pose3d robotPose = questPose.transformBy(VisionConstants.QuestNavConstants.ROBOT_TO_QUEST.inverse());
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
        questNav.setPose(pose);
    }

    /**
     * Returns a supplier for the current robot pose from QuestNav as a Pose3d.
     *
     * @return A supplier that provides the current robot pose in 3D field coordinates.
     */
    public Supplier<Pose2d> getQuestNavPoseSupplier() {
        return () -> questPose
                .transformBy(VisionConstants.QuestNavConstants.ROBOT_TO_QUEST.inverse())
                .toPose2d();
    }
}
