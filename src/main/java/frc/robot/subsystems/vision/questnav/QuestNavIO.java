package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

public interface QuestNavIO {

    public default void resetQuestNavPose(Pose3d robotPose) {}

    public default void zeroQuestNav() {}

    public default void periodic() {}

    public default void setQuestNavStartPose(Pose3d pose) {}

    public default Supplier<Pose2d> getQuestNavPoseSupplier() {
        return null;
    }
}
