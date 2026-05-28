package frc.robot.autonomy;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;

/** Factory and configuration owner for the autonomous robot brain. */
public class AutonomyManager {
    private final Drive drive;
    private final Vision vision;
    private final Superstructure superstructure;
    private final Intake intake;
    private final Indexer indexer;
    private final RobotState robotState;
    private final AutonomyConfig config = new AutonomyConfig();

    public AutonomyManager(
            Drive drive,
            Vision vision,
            Superstructure superstructure,
            Intake intake,
            Indexer indexer,
            RobotState robotState) {
        this.drive = drive;
        this.vision = vision;
        this.superstructure = superstructure;
        this.intake = intake;
        this.indexer = indexer;
        this.robotState = robotState;
    }

    public Command createBrainCommand(boolean finishAfterOneScoredCycle) {
        return new AutonomousRobotBrain(
                drive, vision, superstructure, intake, indexer, robotState, config, finishAfterOneScoredCycle);
    }
}
