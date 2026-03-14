package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

public interface QuestNavIO {

    default void resetQuestNavPose(Pose3d robotPose) {}

    default void zeroQuestNav() {}

    default void periodic() {}

    default void setQuestNavStartPose(Pose3d pose) {}

    default Supplier<Pose2d> getQuestNavPoseSupplier() {
        return null;
    }
}
