package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class PathPlannerCommands {
    private static final LinearVelocity kMaxVel = MetersPerSecond.of(4.0);
    private static final LinearAcceleration kMaxAccel = MetersPerSecondPerSecond.of(3.75);
    private static final AngularVelocity kMaxAngVel = RotationsPerSecond.of(540.0 / 360.0);
    private static final AngularAcceleration kMaxAngAccel = RotationsPerSecondPerSecond.of(720.0 / 360.0);
    private static final LinearVelocity kZeroGoalEndVel = MetersPerSecond.zero();

    private PathPlannerCommands() {}

    public static PathConstraints defaultConstraints() {
        return new PathConstraints(
                kMaxVel.in(MetersPerSecond),
                kMaxAccel.in(MetersPerSecondPerSecond),
                kMaxAngVel.in(RadiansPerSecond),
                kMaxAngAccel.in(RadiansPerSecondPerSecond));
    }

    public static PathConstraints constraintsFromDrive(Drive drive) {
        return new PathConstraints(
                drive.getMaxLinearSpeedMetersPerSec(),
                kMaxAccel.in(MetersPerSecondPerSecond),
                drive.getMaxAngularSpeedRadPerSec(),
                kMaxAngAccel.in(RadiansPerSecondPerSecond));
    }

    public static Command pathfindToPose(Pose2d targetPose) {
        return pathfindToPose(targetPose, defaultConstraints(), kZeroGoalEndVel);
    }

    public static Command pathfindToPose(double xMeters, double yMeters, Rotation2d rotation) {
        return pathfindToPose(new Pose2d(xMeters, yMeters, rotation));
    }

    public static Command pathfindToPose(
            Pose2d targetPose, PathConstraints constraints, LinearVelocity goalEndVelocity) {
        return AutoBuilder.pathfindToPose(targetPose, constraints, goalEndVelocity)
                .withName("PathfindToPose");
    }

    public static Command pathfindToPose(Drive drive, Pose2d targetPose) {
        return pathfindToPose(targetPose, constraintsFromDrive(drive), kZeroGoalEndVel);
    }

    // public static Command pathfindThenFollowPath(String pathName) {
    //     return pathfindThenFollowPath(pathName, defaultConstraints());
    // }

    // public static Command pathfindThenFollowPath(String pathName, PathConstraints pathfindingConstraints) {
    //     PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    //     return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints)
    //             .withName("PathfindThenFollow-" + pathName);
    // }
}
