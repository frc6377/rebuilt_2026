package frc.robot.FullAuto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.GamePieceCameraSim;
import org.ironmaple.simulation.IntakeSimulation;

public class FullAutoSim extends FullAutoIO {
    private final GamePieceCameraSim camera;

    public FullAutoSim(RobotContainer robot) {
        super(robot);
        camera = robot.getGamePieceCamera();
    }

    @Override
    protected Pose3d[] getVisiblePieces() {
        if (camera == null) {
            return new Pose3d[0];
        }
        return camera.getVisiblePieces();
    }

    @Override
    protected boolean intakeHasFuel() {
        IntakeSimulation intakeSim = robot.getIntakeSimulation();
        return intakeSim != null && intakeSim.getGamePiecesAmount() > 0;
    }

    @Override
    protected boolean consumeIntakeFuel() {
        IntakeSimulation intakeSim = robot.getIntakeSimulation();
        return intakeSim != null && intakeSim.obtainGamePieceFromIntake();
    }

    @Override
    protected int getActualIntakeFuel() {
        IntakeSimulation intakeSim = robot.getIntakeSimulation();
        return intakeSim != null ? intakeSim.getGamePiecesAmount() : 0;
    }

    @Override
    protected Command scoringFireCommand() {
        return superstructure.simAutoFireFromIntakeHoldCommand(
                () -> true, this::intakeHasFuel, this::consumeIntakeFuel);
    }
}
