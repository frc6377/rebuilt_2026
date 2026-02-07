// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCalibrationCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOKrakenX60;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.shooter.GamePieceTrajectorySimulation;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.left.LeftShooter;
import frc.robot.subsystems.shooter.left.LeftShooterIO;
import frc.robot.subsystems.shooter.left.LeftShooterIOKrakenX60;
import frc.robot.subsystems.shooter.left.LeftShooterIOSim;
import frc.robot.subsystems.shooter.right.RightShooter;
import frc.robot.subsystems.shooter.right.RightShooterIO;
import frc.robot.subsystems.shooter.right.RightShooterIOKrakenX60;
import frc.robot.subsystems.shooter.right.RightShooterIOSim;
import frc.robot.subsystems.state.RobotState;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Superstructure subsystem that owns the shooter and hood. */
public class Superstructure extends SubsystemBase {
    // Trajectory target heights (tunable via NetworkTables)
    private static final LoggedNetworkNumber maxHeightFeet =
            new LoggedNetworkNumber("Shooting/MaxHeightFeet", ShooterConstants.defaultMaxHeightFeet);
    private static final LoggedNetworkNumber targetHeightFeet =
            new LoggedNetworkNumber("Shooting/TargetHeightFeet", ShooterConstants.defaultTargetHeightFeet);

    // Fine-tuning offsets
    private static final LoggedNetworkNumber hoodAngleOffset =
            new LoggedNetworkNumber("Shooting/HoodAngleOffset", ShooterConstants.defaultHoodAngleOffset);
    private static final LoggedNetworkNumber rpmMultiplier =
            new LoggedNetworkNumber("Shooting/RPMMultiplier", ShooterConstants.defaultRpmMultiplier);

    private final LeftShooter leftShooter;
    private final RightShooter rightShooter;
    private final Hood hood;
    private final RobotState robotState;
    private GamePieceTrajectorySimulation gamePieceTrajectorySimulation;

    /** Creates the superstructure and selects IO implementations by mode. */
    public Superstructure() {
        RobotState createdState = RobotState.getInstance();
        if (createdState == null) {
            createdState = RobotState.create();
        }
        this.robotState = createdState;

        LeftShooterIO leftShooterIO;
        RightShooterIO rightShooterIO;
        HoodIO hoodIO;

        switch (Constants.currentMode) {
            case REAL:
                leftShooterIO = new LeftShooterIOKrakenX60();
                rightShooterIO = new RightShooterIOKrakenX60();
                hoodIO = ShooterConstants.hoodEnabled ? new HoodIOKrakenX60() : null;
                break;
            case SIM:
                leftShooterIO = new LeftShooterIOSim();
                rightShooterIO = new RightShooterIOSim();
                hoodIO = ShooterConstants.hoodEnabled ? new HoodIOSim() : null;
                break;
            default:
                leftShooterIO = new LeftShooterIO() {};
                rightShooterIO = new RightShooterIO() {};
                hoodIO = null;
                break;
        }

        this.leftShooter = new LeftShooter(leftShooterIO);
        this.rightShooter = new RightShooter(rightShooterIO);
        this.hood = hoodIO != null ? new Hood(hoodIO) : null;
    }

    @Override
    public void periodic() {
        if (gamePieceTrajectorySimulation == null) {
            return;
        }

        int desiredCount = robotState.getSimGamePieceCount();
        if (gamePieceTrajectorySimulation.getBallsInHopper() != desiredCount) {
            gamePieceTrajectorySimulation.setBallsInHopper(desiredCount);
        }

        gamePieceTrajectorySimulation.updateAutoFire();
        robotState.setSimGamePieceCount(gamePieceTrajectorySimulation.getBallsInHopper());
    }

    /** Configure the game piece trajectory simulation (SIM mode only). */
    public void configureGamePieceSimulation(SwerveDriveSimulation driveSimulation) {
        if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
            return;
        }

        gamePieceTrajectorySimulation = new GamePieceTrajectorySimulation(
                driveSimulation, () -> getAverageFlywheelVelocity().in(RPM), () -> getHoodAngle()
                        .in(Degrees));
        robotState.setSimGamePieceCount(gamePieceTrajectorySimulation.getBallsInHopper());
    }

    public GamePieceTrajectorySimulation getGamePieceTrajectorySimulation() {
        return gamePieceTrajectorySimulation;
    }

    public boolean hasGamePieceTrajectorySimulation() {
        return gamePieceTrajectorySimulation != null;
    }

    public Command simAutoFireHoldCommand(BooleanSupplier indexerRunningSupplier) {
        if (gamePieceTrajectorySimulation == null) {
            return Commands.none();
        }

        return Commands.startEnd(() -> gamePieceTrajectorySimulation.enableAutoFire(indexerRunningSupplier), () -> {
                    gamePieceTrajectorySimulation.setAutoFireEnabled(false);
                    gamePieceTrajectorySimulation.setIndexerRunningSupplier(() -> false);
                })
                .withName("SimAutoFireHold");
    }

    public boolean simShouldIndexerRun() {
        return gamePieceTrajectorySimulation != null && gamePieceTrajectorySimulation.shouldIndexerRun();
    }

    public Command simLaunchGamePieceCommand() {
        if (gamePieceTrajectorySimulation == null) {
            return Commands.none();
        }

        return Commands.runOnce(() -> SimulatedArena.getInstance()
                        .addGamePieceProjectile(gamePieceTrajectorySimulation.launchGamePiece()))
                .withName("SimLaunchGamePiece");
    }

    public Command simAddBallsCommand(int count) {
        if (gamePieceTrajectorySimulation == null) {
            return Commands.none();
        }

        return Commands.runOnce(() -> gamePieceTrajectorySimulation.addBalls(count))
                .withName("SimAddBalls");
    }

    public Command simSetAutoFireEnabledCommand(boolean enabled) {
        if (gamePieceTrajectorySimulation == null) {
            return Commands.none();
        }

        return Commands.runOnce(() -> gamePieceTrajectorySimulation.setAutoFireEnabled(enabled))
                .withName("SimAutoFireEnabled:" + enabled);
    }

    public Command createShooterCalibrationCommand(
            SwerveDriveSimulation driveSimulation, Consumer<Pose2d> poseResetter) {
        if (gamePieceTrajectorySimulation == null || hood == null || driveSimulation == null) {
            return null;
        }

        return new ShooterCalibrationCommand(
                hood, leftShooter, gamePieceTrajectorySimulation, driveSimulation, poseResetter);
    }

    public LeftShooter getLeftShooter() {
        return leftShooter;
    }

    public RightShooter getRightShooter() {
        return rightShooter;
    }

    public Hood getHood() {
        return hood;
    }

    public boolean hasHood() {
        return hood != null;
    }

    public Angle getHoodAngle() {
        return hood != null ? hood.getAngle() : ShooterConstants.fixedHoodAngle;
    }

    public void setHoodAngle(Angle angle) {
        if (hood != null && ShooterConstants.hoodEnabled) {
            hood.setAngle(angle);
        }
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        leftShooter.setFlywheelVelocity(velocity);
        rightShooter.setFlywheelVelocity(velocity);
    }

    public AngularVelocity getLeftFlywheelVelocity() {
        return leftShooter.getFlywheelVelocity();
    }

    public AngularVelocity getRightFlywheelVelocity() {
        return rightShooter.getFlywheelVelocity();
    }

    public AngularVelocity getAverageFlywheelVelocity() {
        double rpm =
                (getLeftFlywheelVelocity().in(RPM) + getRightFlywheelVelocity().in(RPM)) / 2.0;
        return RPM.of(rpm);
    }

    public void stopShooter() {
        leftShooter.stop();
        rightShooter.stop();
    }

    public Command stopShooterCommand() {
        return leftShooter.stopCommand().alongWith(rightShooter.stopCommand());
    }

    /** Record to hold calculated shooting parameters with type-safe units. */
    public record ShootingParameters(Angle hoodAngle, AngularVelocity flywheelVelocity) {}

    /** Calculate the distance from the robot to the hub. */
    public Distance getDistanceToHub(Pose2d robotPose) {
        Translation2d hubPosition = FieldConstants.getHubPosition();
        return Meters.of(robotPose.getTranslation().getDistance(hubPosition));
    }

    /** Calculate the angle from the robot to the hub. */
    public Rotation2d getAngleToHub(Pose2d robotPose) {
        Translation2d hubPosition = FieldConstants.getHubPosition();
        Translation2d toHub = hubPosition.minus(robotPose.getTranslation());
        return new Rotation2d(toHub.getX(), toHub.getY());
    }

    /** Calculate the angle from the robot to the alliance wall center. */
    public Rotation2d getAngleToAllianceWall(Pose2d robotPose) {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        double targetX = isRed ? FieldConstants.FIELD_LENGTH.in(Meters) : 0.0;
        double targetY = FieldConstants.FIELD_WIDTH.in(Meters) / 2.0;
        Translation2d target = new Translation2d(targetX, targetY);
        Translation2d toTarget = target.minus(robotPose.getTranslation());
        return new Rotation2d(toTarget.getX(), toTarget.getY());
    }

    /**
     * Calculate shooting parameters for a trajectory that: 1. Starts at shooter height 2. Peaks at maxHeight (default
     * 8ft) 3. Lands at targetHeight (default 6ft / hub opening)
     */
    public ShootingParameters calculateShootingParameters(Distance distance) {
        if (!ShooterConstants.hoodEnabled) {
            return calculateParametersForFixedAngle(distance, ShooterConstants.fixedHoodAngle);
        }

        Distance maxHeight = Feet.of(maxHeightFeet.get());
        Distance targetHeight = Feet.of(targetHeightFeet.get());
        Distance startHeight = ShooterConstants.shooterHeight;

        Distance riseHeight = maxHeight.minus(startHeight);
        Distance fallHeight = maxHeight.minus(targetHeight);

        if (riseHeight.in(Meters) <= 0 || fallHeight.in(Meters) < 0) {
            return new ShootingParameters(Degrees.of(45.0), ShooterConstants.minShootingFlywheelVelocity);
        }

        double gravityMps2 = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
        double riseMeters = riseHeight.in(Meters);
        double fallMeters = fallHeight.in(Meters);
        double distanceMeters = distance.in(Meters);

        double timeToRise = Math.sqrt(2.0 * riseMeters / gravityMps2);
        double timeToFall = Math.sqrt(2.0 * fallMeters / gravityMps2);
        double totalTime = timeToRise + timeToFall;

        LinearVelocity horizontalVelocity = MetersPerSecond.of(distanceMeters / totalTime);
        LinearVelocity verticalVelocity = MetersPerSecond.of(gravityMps2 * timeToRise);

        double vx = horizontalVelocity.in(MetersPerSecond);
        double vy = verticalVelocity.in(MetersPerSecond);
        Angle launchAngle = Radians.of(Math.atan2(vy, vx));
        LinearVelocity launchSpeed = MetersPerSecond.of(Math.sqrt(vx * vx + vy * vy));

        Angle hoodAngle = Degrees.of(launchAngle.in(Degrees) + hoodAngleOffset.get());

        double hoodDegrees = hoodAngle.in(Degrees);
        hoodDegrees = Math.max(
                ShooterConstants.minHoodAngle.in(Degrees),
                Math.min(ShooterConstants.maxHoodAngle.in(Degrees), hoodDegrees));
        hoodAngle = Degrees.of(hoodDegrees);

        double flywheelRadiusMeters = ShooterConstants.flywheelRadius.in(Meters);
        double launchSpeedMps = launchSpeed.in(MetersPerSecond);
        double angularVelocityRadPerSec = (launchSpeedMps / ShooterConstants.launchEfficiency) / flywheelRadiusMeters;
        AngularVelocity flywheelVelocity = RadiansPerSecond.of(angularVelocityRadPerSec * rpmMultiplier.get());

        double rpm = flywheelVelocity.in(RPM);
        rpm = Math.max(
                ShooterConstants.minShootingFlywheelVelocity.in(RPM),
                Math.min(ShooterConstants.maxShootingFlywheelVelocity.in(RPM), rpm));
        flywheelVelocity = RPM.of(rpm);

        return new ShootingParameters(hoodAngle, flywheelVelocity);
    }

    private ShootingParameters calculateParametersForFixedAngle(Distance distance, Angle fixedAngle) {
        double distanceMeters = distance.in(Meters);
        double angleRadians = fixedAngle.in(Radians);
        double gravityMps2 = ShooterConstants.gravity.in(MetersPerSecondPerSecond);

        double sin2Theta = Math.sin(2.0 * angleRadians);
        if (Math.abs(sin2Theta) < 0.001) {
            return new ShootingParameters(fixedAngle, ShooterConstants.maxShootingFlywheelVelocity);
        }

        double launchSpeedMps = Math.sqrt(distanceMeters * gravityMps2 / sin2Theta);
        launchSpeedMps *= rpmMultiplier.get();

        double flywheelRadiusMeters = ShooterConstants.flywheelRadius.in(Meters);
        double angularVelocityRadPerSec = (launchSpeedMps / ShooterConstants.launchEfficiency) / flywheelRadiusMeters;
        AngularVelocity flywheelVelocity = RadiansPerSecond.of(angularVelocityRadPerSec);

        double rpm = flywheelVelocity.in(RPM);
        rpm = Math.max(
                ShooterConstants.minShootingFlywheelVelocity.in(RPM),
                Math.min(ShooterConstants.maxShootingFlywheelVelocity.in(RPM), rpm));
        flywheelVelocity = RPM.of(rpm);

        return new ShootingParameters(fixedAngle, flywheelVelocity);
    }

    /** Command that continuously updates hood angle and flywheel speed based on distance to hub. */
    public Command autoSpeedShooter(Supplier<Pose2d> poseSupplier) {
        var requirements = hood != null
                ? new SubsystemBase[] {hood, leftShooter, rightShooter}
                : new SubsystemBase[] {leftShooter, rightShooter};

        return Commands.run(
                        () -> {
                            Pose2d robotPose = poseSupplier.get();
                            Distance distance = getDistanceToHub(robotPose);
                            ShootingParameters params = calculateShootingParameters(distance);
                            Angle hoodAngle = params.hoodAngle();
                            AngularVelocity flywheelVelocity = params.flywheelVelocity();

                            Logger.recordOutput("Shooting/DistanceToHub", distance.in(Meters));
                            Logger.recordOutput("Shooting/CalculatedHoodAngle", hoodAngle.in(Degrees));
                            Logger.recordOutput("Shooting/CalculatedRPM", flywheelVelocity.in(RPM));

                            setHoodAngle(hoodAngle);
                            setFlywheelVelocity(flywheelVelocity);
                        },
                        requirements)
                .withName("AutoAimShooter");
    }

    /** Command that aims the robot at the hub while driving. */
    public Command aimAtHubWhileDriving(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> getAngleToHub(drive.getPose()))
                .withName("AimAtHub");
    }

    /** Full auto-aim command: aims robot at hub AND sets hood/flywheel automatically. */
    public Command fullAutoAim(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        Command command = aimAtHubWhileDriving(drive, xSupplier, ySupplier)
                .alongWith(autoSpeedShooter(drive::getPose))
                .withName("FullAutoAim");
        return command.beforeStarting(() -> robotState.setMode(RobotState.Mode.SHOOTING))
                .finallyDo(() -> robotState.setMode(RobotState.Mode.IDLE));
    }

    /** Spins up the shooter and aims based on RobotState. */
    public Command spinUpShooterCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        Supplier<Rotation2d> angleSupplier = () -> {
            Pose2d pose = drive.getPose();
            if (robotState.getMode() == RobotState.Mode.SHOOTING) {
                return getAngleToHub(pose);
            }
            if (robotState.isShuttling()) {
                return getAngleToAllianceWall(pose);
            }
            return drive.getRotation();
        };

        return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, angleSupplier)
                .alongWith(autoSpeedShooter(drive::getPose))
                .withName("SpinUpShooter");
    }

    public boolean atTargetVelocity() {
        return leftShooter.atTargetVelocity() && rightShooter.atTargetVelocity();
    }
}
