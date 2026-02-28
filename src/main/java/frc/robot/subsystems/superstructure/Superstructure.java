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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCalibrationCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOKrakenX60;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.left.LeftShooter;
import frc.robot.subsystems.shooter.right.RightShooter;
import frc.robot.subsystems.upgoer.Upgoer;
import frc.robot.subsystems.upgoer.UpgoerConstants;
import frc.robot.subsystems.upgoer.UpgoerIO;
import frc.robot.subsystems.upgoer.UpgoerIOKrakenX60;
import frc.robot.subsystems.upgoer.UpgoerIOSim;
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
    private static final LoggedNetworkNumber calculationMode =
            new LoggedNetworkNumber("Shooting/CalculationMode", ShooterConstants.kDefaultCalculationMode.ordinal());
    private static final LoggedNetworkNumber manualShootingSpeedRPM =
            new LoggedNetworkNumber("Shooting/ManualShootingSpeedRPM", ShooterConstants.kManualShootingSpeedRPM);
    private static final LoggedNetworkNumber manualShootingEnabled = new LoggedNetworkNumber(
            "Shooting/ManualShootingEnabled", ShooterConstants.kManualShootingEnabled ? 1.0 : 0.0);
    // Testing / Bench Mode
    private static final LoggedNetworkNumber benchModeEnabled =
            new LoggedNetworkNumber("Shooting/BenchMode/Enabled", ShooterConstants.defaultBenchModeEnabled);
    private static final LoggedNetworkNumber benchModeDistanceFeet = new LoggedNetworkNumber(
            "Shooting/BenchMode/DistanceMeters>", ShooterConstants.defaultBenchModeDistanceMeters);

    private final Shooter shooter;
    private final Hood hood;
    private final Upgoer upgoer;
    private final Indexer indexer;
    private final RobotState robotState;
    private GamePieceTrajectorySimulation gamePieceTrajectorySimulation;

    /** Creates the superstructure and selects IO implementations by mode. */
    public Superstructure(BooleanSupplier isIntaking) {
        RobotState createdState = RobotState.getInstance();
        if (createdState == null) {
            createdState = RobotState.create();
        }
        this.robotState = createdState;

        this.shooter = new Shooter();

        HoodIO hoodIO;
        UpgoerIO upgoerIO;
        IndexerIO indexerIO;

        switch (Constants.currentMode) {
            case REAL:
                hoodIO = Constants.EnabledSubsystems.kHood ? new HoodIOKrakenX60() : new HoodIO() {};
                upgoerIO = Constants.EnabledSubsystems.kUpgoer ? new UpgoerIOKrakenX60() : new UpgoerIO() {};
                indexerIO = Constants.EnabledSubsystems.kIndexer ? new IndexerIOReal() : new IndexerIO() {};
                break;
            case SIM:
                hoodIO = Constants.EnabledSubsystems.kHood ? new HoodIOSim() : new HoodIO() {};
                upgoerIO = Constants.EnabledSubsystems.kUpgoer ? new UpgoerIOSim() : new UpgoerIO() {};
                indexerIO = Constants.EnabledSubsystems.kIndexer ? new IndexerIOSim() : new IndexerIO() {};
                break;
            default:
                hoodIO = new HoodIO() {};
                upgoerIO = new UpgoerIO() {};
                indexerIO = new IndexerIO() {};
                break;
        }

        this.hood = new Hood(hoodIO);
        this.upgoer = new Upgoer(upgoerIO);
        this.indexer = new Indexer(indexerIO);

        indexer.setDefaultCommand(
                Commands.run(() -> indexer.setRunning(shooter.isRunning() || isIntaking.getAsBoolean()), indexer));
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
        if (gamePieceTrajectorySimulation == null || !Constants.EnabledSubsystems.kHood || driveSimulation == null) {
            return null;
        }

        return new ShooterCalibrationCommand(
                hood, shooter.getLeft(), gamePieceTrajectorySimulation, driveSimulation, poseResetter);
    }

    public LeftShooter getLeftShooter() {
        return shooter.getLeft();
    }

    public RightShooter getRightShooter() {
        return shooter.getRight();
    }

    public Hood getHood() {
        return hood;
    }

    public Upgoer getUpgoer() {
        return upgoer;
    }

    public boolean hasHood() {
        return !Constants.EnabledSubsystems.kHood;
    }

    public Angle getHoodAngle() {
        return !Constants.EnabledSubsystems.kHood ? hood.getAngle() : ShooterConstants.kFixedHoodAngle;
    }

    public void setHoodAngle(Angle angle) {
        if (hood != null) {
            hood.setAngle(angle);
        }
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        shooter.setFlywheelVelocity(velocity);
    }

    public void setUpgoerVelocity(AngularVelocity velocity) {
        upgoer.setVelocity(velocity);
    }

    public AngularVelocity getLeftFlywheelVelocity() {
        return shooter.getLeft().getFlywheelVelocity();
    }

    public AngularVelocity getRightFlywheelVelocity() {
        return shooter.getRight().getFlywheelVelocity();
    }

    public AngularVelocity getAverageFlywheelVelocity() {
        double rpm =
                (getLeftFlywheelVelocity().in(RPM) + getRightFlywheelVelocity().in(RPM)) / 2.0;
        return RPM.of(rpm);
    }

    public void stopShooter() {
        shooter.stop();
    }

    public void stopUpgoer() {
        upgoer.stop();
    }

    public Command stopShooterCommand() {
        return shooter.stopCommand();
    }

    public Command stopUpgoerCommand() {
        return upgoer.stopCommand();
    }

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
        double targetX = isRed ? FieldConstants.fieldLength : 0.0;
        double targetY = FieldConstants.fieldWidth / 2.0;
        Translation2d target = new Translation2d(targetX, targetY);
        Translation2d toTarget = target.minus(robotPose.getTranslation());
        return new Rotation2d(toTarget.getX(), toTarget.getY());
    }

    public boolean isInShootingZone(Pose2d robotPose) {
        double fieldLengthMeters = FieldConstants.fieldLength;
        double xMeters = robotPose.getTranslation().getX();
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        double distanceFromOwnWall = isRed ? fieldLengthMeters - xMeters : xMeters;
        // Can shoot from own half of the field
        return distanceFromOwnWall <= fieldLengthMeters / 2.0;
    }

    /** Command that continuously updates hood angle and flywheel speed based on distance to hub. */
    public Command autoSpeedShooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> velocitySupplier) {
        return Commands.run(
                        () -> {
                            Pose2d robotPose;
                            ChassisSpeeds speeds;

                            if (benchModeEnabled.get() > 0.5) {
                                // Bench mode: Calculate a virtual pose at the specified distance
                                double distMeters =
                                        Feet.of(benchModeDistanceFeet.get()).in(Meters);
                                // Hub is usually at (FieldLength - distance, FieldWidth/2) or similar.
                                // We'll just mock a static pose at that distance from the hub.
                                Translation2d hubPos = FieldConstants.getHubPosition();
                                robotPose = new Pose2d(hubPos.getX() - distMeters, hubPos.getY(), new Rotation2d());
                                speeds = new ChassisSpeeds();
                            } else {
                                robotPose = poseSupplier.get();
                                speeds = velocitySupplier.get();
                            }

                            boolean inZone = isInShootingZone(robotPose);

                            Logger.recordOutput("Shooting/InShootingZone", inZone);

                            if (inZone) {
                                TrajectoryBall.ShootingParameters params = TrajectoryBall.calculate(
                                        ShooterConstants.CalculationMode.values()[(int) calculationMode.get()],
                                        hasHood(),
                                        robotPose,
                                        speeds,
                                        Feet.of(maxHeightFeet.get()),
                                        Feet.of(targetHeightFeet.get()),
                                        hoodAngleOffset.get(),
                                        rpmMultiplier.get(),
                                        ShooterConstants.kSotfEnabled);

                                Angle hoodAngle = params.hoodAngle();
                                AngularVelocity flywheelVelocity = params.flywheelVelocity();

                                Logger.recordOutput(
                                        "Shooting/DistanceToHub",
                                        getDistanceToHub(robotPose).in(Meters));
                                Logger.recordOutput("Shooting/CalculatedHoodAngle", hoodAngle.in(Degrees));
                                Logger.recordOutput("Shooting/CalculatedRPM", flywheelVelocity.in(RPM));
                                Logger.recordOutput("Shooting/TargetHeading", params.targetHeading());

                                setHoodAngle(hoodAngle);
                                setFlywheelVelocity(flywheelVelocity);
                            } else {
                                // Outside shooting zone — idle the shooter
                                Logger.recordOutput("Shooting/CalculatedHoodAngle", 0.0);
                                Logger.recordOutput("Shooting/CalculatedRPM", 0.0);
                                stopShooter();
                            }
                        },
                        hood,
                        shooter.getLeft(),
                        shooter.getRight())
                .withName("AutoAimShooter");
    }

    public Command autoSpeedShooter(Supplier<Pose2d> poseSupplier) {
        return autoSpeedShooter(poseSupplier, ChassisSpeeds::new);
    }

    public Command autoSpeedShooter() {
        return autoSpeedShooter(Pose2d::new, ChassisSpeeds::new);
    }
    /** Command that aims the robot at the hub while driving. */
    public Command aimAtHubWhileDriving(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> getAngleToHub(drive.getPose()))
                .withName("AimAtHub");
    }

    /** Command that fires the shooter (feeds upgoer only when flywheels are at speed). */
    public Command fireCommand() {
        return Commands.run(
                        () -> {
                            upgoer.setVelocity(UpgoerConstants.defaultFeedVelocity);
                        },
                        upgoer)
                .withName("SuperstructureFire");
    }

    public Command unjamCommand() {
        return Commands.run(
                        () -> {
                            upgoer.setVelocity(UpgoerConstants.defaultUnjamVelocity);
                        },
                        upgoer)
                .withName("SuperstructureUnjam");
    }

    /** Full auto-aim command: aims robot at hub AND sets hood/flywheel automatically. */
    public Command fullAutoAim(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return aimAtHubWhileDriving(drive, xSupplier, ySupplier)
                .alongWith(autoSpeedShooter(drive::getPose, drive::getChassisSpeeds))
                .withName("FullAutoAim")
                .beforeStarting(() -> robotState.setMode(RobotState.Mode.SHOOTING))
                .finallyDo(() -> robotState.setMode(RobotState.Mode.IDLE));
    }

    public Command spinUpShooterCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        Supplier<Rotation2d> angleSupplier = () -> {
            Pose2d pose = drive.getPose();
            if (isInShootingZone(pose)) {
                return getAngleToHub(pose);
            }
            // On opponent's side —> shuttle back towards own wall
            return getAngleToAllianceWall(pose);
        };

        return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, angleSupplier)
                .alongWith(autoSpeedShooter(drive::getPose, drive::getChassisSpeeds))
                .withName("SpinUpShooter");
    }

    public boolean atTargetVelocity() {
        return shooter.getLeft().atTargetVelocity() && shooter.getRight().atTargetVelocity();
    }

    public Command setFlywheelVelocityCommand(AngularVelocity velocity) {
        return Commands.runOnce(() -> setFlywheelVelocity(velocity), shooter.getLeft(), shooter.getRight())
                .withName("SetFlywheelVelocity:" + velocity.in(RPM) + "RPM");
    }

    public Command setFlywheelVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
        return Commands.run(() -> setFlywheelVelocity(velocitySupplier.get()), shooter.getLeft(), shooter.getRight())
                .withName("SetFlywheelVelocity");
    }

    public Command setFlywheelVelocityAndWaitCommand(AngularVelocity velocity) {
        return setFlywheelVelocityCommand(velocity).until(this::atTargetVelocity);
    }

    public Command autoChooseShootingCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        if (ShooterConstants.kManualShootingEnabled) {
            return runOnce(() -> setFlywheelVelocity(RPM.of(manualShootingSpeedRPM.get())));
        } else {
            return fullAutoAim(drive, xSupplier, ySupplier);
        }
    }
}
