package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class QuestNavSubsystem extends SubsystemBase {
    private final QuestNav questNav;
    private final Drive drivetrain;
    private Pose3d robotPose;
    private Pose2d initialPose;
    private Pose3d questPose;
    private static final Transform3d ROBOT_TO_QUEST = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    private static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);

    public QuestNavSubsystem(Drive drive) {
        questNav = new QuestNav();
        robotPose = new Pose3d();
        questPose = robotPose.transformBy(ROBOT_TO_QUEST);
        questNav.setPose(questPose);

        this.drivetrain = drive;
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic(); // REQUIRED - Must be called every periodic cycle
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
        for (PoseFrame questFrame : questFrames) {
            if (questFrame.isTracking()) {
                questPose = questFrame.questPose3d();
                double timestamp = questFrame.dataTimestamp();
                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

                drivetrain.accept(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            }
        }

        Logger.recordOutput("Vision/online", questNav.isConnected());
        Logger.recordOutput("Vision/QuestNav", questPose);
    }

    public void resetPose(Pose3d robotPose) {
        Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
        questNav.setPose(questPose);
    }

    public void setStartPose(Pose2d initialPose) {
        this.initialPose = initialPose;
    }

    /**
     * Returns a supplier for the current robot pose from QuestNav as a Pose2d.
     *
     * @return A supplier that provides the current robot pose in 2D field coordinates
     */
    public Supplier<Pose2d> getPoseSupplier() {
        return () -> questPose.transformBy(ROBOT_TO_QUEST.inverse()).toPose2d();
    }
}
