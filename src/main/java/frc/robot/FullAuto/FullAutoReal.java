package frc.robot.FullAuto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class FullAutoReal extends FullAutoIO {
    public FullAutoReal(RobotContainer robot) {
        super(robot);
    }

    @Override
    protected Pose3d[] getVisiblePieces() {
        return new Pose3d[0];
    }

    @Override
    protected boolean intakeHasFuel() {
        return false;
    }

    @Override
    protected boolean consumeIntakeFuel() {
        return false;
    }

    @Override
    protected int getActualIntakeFuel() {
        return 0;
    }

    @Override
    protected Command scoringFireCommand() {
        return Commands.none();
    }
}
