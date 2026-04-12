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
import static java.lang.Math.round;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCalibrationCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.shooter.BaseShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.upgoer.Upgoer;
import frc.robot.subsystems.upgoer.UpgoerConstants;
import frc.robot.subsystems.upgoer.UpgoerIO;
import frc.robot.subsystems.upgoer.UpgoerIOKrakenX60;
import frc.robot.subsystems.upgoer.UpgoerIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OILayer.OI;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Superstructure subsystem that owns the shooter. */
public class Superstructure extends SubsystemBase {
    // Trajectory target heights (tunable via NetworkTables)
    private static final LoggedNetworkNumber maxHeightFeet =
            new LoggedNetworkNumber("Shooting/MaxHeightFeet", ShooterConstants.defaultMaxHeightFeet);
    private static final LoggedNetworkNumber targetHeightFeet =
            new LoggedNetworkNumber("Shooting/TargetHeightFeet", ShooterConstants.defaultTargetHeightFeet);

    // Fine-tuning offsets
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
            "Shooting/BenchMode/DistanceMeters", ShooterConstants.defaultBenchModeDistanceMeters);

    private final Shooter shooter;
    private final Upgoer leftUpgoer;
    private final Upgoer rightUpgoer;
    private final Indexer indexer;
    private final Vision vision;
    private final OI oi;
    private final RobotState robotState;
    private GamePieceTrajectorySimulation gamePieceTrajectorySimulation;
    private AngularVelocity manualShootingVelocity = RPM.of(ShooterConstants.kManualShootingSpeedRPM);
    private Command currentShootingCommand;
    private TrajectoryBall.ShootingParameters latestParameters = null;

    /** Creates the superstructure and selects IO implementations by mode. */
    public Superstructure(BooleanSupplier isIntaking, Vision vision, OI oi) {
        RobotState createdState = RobotState.getInstance();
        if (createdState == null) {
            createdState = RobotState.create();
        }
        this.robotState = createdState;
        this.vision = vision;

        this.shooter = new Shooter();
        this.oi = oi;
        UpgoerIO leftUpgoerIO;
        UpgoerIO rightUpgoerIO;
        IndexerIO indexerIO;

        switch (Constants.currentMode) {
            case REAL:
                leftUpgoerIO = Constants.EnabledSubsystems.kShooterUpgoerLeft
                        ? new UpgoerIOKrakenX60(
                                Constants.CANIDs.MotorIDs.kLeftUpgoerMotorCANID, "LeftShooterUpgoer", -1)
                        : new UpgoerIO() {};
                rightUpgoerIO = Constants.EnabledSubsystems.kShooterUpgoerRight
                        ? new UpgoerIOKrakenX60(
                                Constants.CANIDs.MotorIDs.kRightUpgoerMotorCANID, "RightShooterUpgoer", 1)
                        : new UpgoerIO() {};
                indexerIO = Constants.EnabledSubsystems.kIndexer ? new IndexerIOReal() : new IndexerIO() {};
                break;
            case SIM:
                leftUpgoerIO = Constants.EnabledSubsystems.kShooterUpgoerLeft
                        ? new UpgoerIOSim(Constants.CANIDs.MotorIDs.kLeftUpgoerMotorCANID, "LeftShooterUpgoer")
                        : new UpgoerIO() {};
                rightUpgoerIO = Constants.EnabledSubsystems.kShooterUpgoerRight
                        ? new UpgoerIOSim(Constants.CANIDs.MotorIDs.kRightUpgoerMotorCANID, "RightShooterUpgoer")
                        : new UpgoerIO() {};
                indexerIO = Constants.EnabledSubsystems.kIndexer ? new IndexerIOSim() : new IndexerIO() {};
                break;
            default:
                leftUpgoerIO = new UpgoerIO() {};
                rightUpgoerIO = new UpgoerIO() {};
                indexerIO = new IndexerIO() {};
                break;
        }

        this.leftUpgoer = new Upgoer(leftUpgoerIO, "LeftShooterUpgoer", 1);
        this.rightUpgoer = new Upgoer(rightUpgoerIO, "RightShooterUpgoer", 1);
        this.indexer = new Indexer(indexerIO);
        this.currentShootingCommand = this.runFlywheelVelocityManual();
    }

    @Override
    public void periodic() {
        // Log hub active state from game data

        boolean isHubActive = FieldConstants.isHubActive();
        Logger.recordOutput("Shooting/HubActive", isHubActive);
        Logger.recordOutput("Shooting/HubFlashing", FieldConstants.isHubFlashing());
        Logger.recordOutput("Shooting/HubIndicatorOn", FieldConstants.isHubIndicatorOn());
        Logger.recordOutput("Shooting/TimeUntilHubStateChange", FieldConstants.getTimeUntilHubStateChange());
        Logger.recordOutput(
                "Shooting/DistanceToHub", round(vision.getHubDistanceMeasure().in(Meters) * 100.0) / 100.0);
        if (FieldConstants.getTimeUntilHubStateChange() > 0 && FieldConstants.getTimeUntilHubStateChange() < 1) {
            CommandScheduler.getInstance().schedule(oi.setRumble(0, 0));
        } else if (FieldConstants.getTimeUntilHubStateChange() > 4.75
                && FieldConstants.getTimeUntilHubStateChange() < 5) {
            CommandScheduler.getInstance().schedule(oi.setRumble(0.7, 0.7));
        } else if (FieldConstants.getTimeUntilHubStateChange() > 4.25
                && FieldConstants.getTimeUntilHubStateChange() < 4.5) {
            CommandScheduler.getInstance().schedule(oi.setRumble(0.75, 0.75));
        } else {
            oi.setRumble(0, 0);
        }

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
                driveSimulation, () -> getAverageFlywheelVelocity().in(RPM));
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
        if (gamePieceTrajectorySimulation == null || driveSimulation == null) {
            return null;
        }

        return new ShooterCalibrationCommand(
                shooter.getLeft(), gamePieceTrajectorySimulation, driveSimulation, poseResetter);
    }

    public BaseShooter getLeftShooter() {
        return shooter.getLeft();
    }

    public BaseShooter getRightShooter() {
        return shooter.getRight();
    }

    public Upgoer getLeftUpgoer() {
        return leftUpgoer;
    }

    public Upgoer getRightUpgoer() {
        return rightUpgoer;
    }

    public Angle getHoodAngle() {
        return ShooterConstants.kFixedHoodAngle;
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        shooter.setFlywheelVelocity(velocity);
    }

    public void setUpgoerVelocity(AngularVelocity velocity) {
        leftUpgoer.setVelocity(velocity);
        rightUpgoer.setVelocity(velocity);
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
        leftUpgoer.stop();
        rightUpgoer.stop();
    }

    public Command stopShooterCommand() {
        return shooter.stopCommand();
    }

    public Command stopUpgoerCommand() {
        return leftUpgoer.stopCommand().alongWith(rightUpgoer.stopCommand());
    }

    /** Calculate the distance from the robot to the hub. */
    public Distance getDistanceToHub(Pose2d robotPose) {
        return Meters.of(robotPose.getTranslation().getDistance(FieldConstants.getHubPosition()));
    }

    /** Calculate the angle from the robot to the hub. */
    public Rotation2d getAngleToHub(Pose2d robotPose) {
        Translation2d toHub = FieldConstants.getHubPosition().minus(robotPose.getTranslation());
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

    private ShooterConstants.CalculationMode getCalculationModeFromDashboard() {
        int modeIndex = (int) Math.round(calculationMode.get());
        ShooterConstants.CalculationMode[] modes = ShooterConstants.CalculationMode.values();
        if (modeIndex < 0 || modeIndex >= modes.length) {
            return ShooterConstants.kDefaultCalculationMode;
        }
        return modes[modeIndex];
    }

    /** Command that continuously updates flywheel speed based on distance to hub. */
    public Command autoSpeedShooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> velocitySupplier) {
        return Commands.run(
                        () -> {
                            Pose2d robotPose;
                            ChassisSpeeds robotSpeeds;
                            if (benchModeEnabled.get() > 0.5) {
                                // Bench mode: use a virtual pose at the configured distance
                                double distMeters =
                                        Feet.of(benchModeDistanceFeet.get()).in(Meters);
                                Translation2d hubPos = FieldConstants.getHubPosition();
                                robotPose = new Pose2d(hubPos.getX() - distMeters, hubPos.getY(), new Rotation2d());
                                robotSpeeds = new ChassisSpeeds();
                            } else {
                                robotPose = poseSupplier.get();
                                robotSpeeds = velocitySupplier.get();
                            }

                            double hubDistanceMeters =
                                    getDistanceToHub(robotPose).in(Meters);
                            latestParameters = TrajectoryBall.calculate(
                                    getCalculationModeFromDashboard(),
                                    robotPose,
                                    robotSpeeds,
                                    Feet.of(maxHeightFeet.get()),
                                    Feet.of(targetHeightFeet.get()),
                                    rpmMultiplier.get(),
                                    ShooterConstants.kSotfEnabled);

                            Logger.recordOutput("Shooting/DistanceSource", "PoseEstimate");
                            Logger.recordOutput("Shooting/OdometryHubDistanceM", getCalculationModeFromDashboard());
                            Logger.recordOutput(
                                    "Shooting/TargetHeadingDeg",
                                    latestParameters.targetHeading().getDegrees());
                            Logger.recordOutput("Shooting/CalculationMode", calculationMode.get());

                            boolean inZone = isInShootingZone(robotPose);
                            Logger.recordOutput("Shooting/InShootingZone", inZone);

                            if (inZone) {
                                Logger.recordOutput("Shooting/DistanceToHub", hubDistanceMeters);
                                Logger.recordOutput(
                                        "Shooting/CalculatedRPM",
                                        latestParameters.flywheelVelocity().in(RPM));

                                setFlywheelVelocity(latestParameters.flywheelVelocity());
                            } else {
                                setFlywheelVelocity(manualShootingVelocity);
                            }
                        },
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
    public Command aimAtHubWhileDriving(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier xModePressed) {
        return DriveCommands.joystickDriveAtAngle(
                        drive, xSupplier, ySupplier, () -> latestParameters.targetHeading(), xModePressed)
                .withName("AimAtHub");
    }

    /** Command that fires the shooter (feeds both upgoers). */
    public Command fireCommand() {
        return Commands.run(
                        () -> {
                            leftUpgoer.setVelocity(UpgoerConstants.defaultFeedVelocity);
                            rightUpgoer.setVelocity(UpgoerConstants.defaultFeedVelocity);
                        },
                        leftUpgoer,
                        rightUpgoer)
                .withName("SuperstructureFire");
    }

    public Command unjamCommand() {
        return Commands.run(
                        () -> {
                            leftUpgoer.setVelocity(UpgoerConstants.defaultUnjamVelocity);
                            rightUpgoer.setVelocity(UpgoerConstants.defaultUnjamVelocity);
                        },
                        leftUpgoer,
                        rightUpgoer)
                .withName("SuperstructureUnjam");
    }

    /** Full auto-aim command: aims robot at hub AND sets flywheel automatically. */
    public Command fullAutoAim(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return aimAtHubWhileDriving(drive, xSupplier, ySupplier, () -> false)
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
        Logger.recordOutput(
                "Shooting/Ready to shoot",
                shooter.getLeft().atTargetVelocity() && shooter.getRight().atTargetVelocity());
        return shooter.getLeft().atTargetVelocity() && shooter.getRight().atTargetVelocity();
    }

    public boolean isReadyToShoot(Rotation2d currentHeading) {
        if (latestParameters == null) return atTargetVelocity();

        boolean flywheelReady = atTargetVelocity(); // atTargetVelocity();
        boolean headingReady =
                Math.abs(currentHeading.minus(latestParameters.targetHeading()).getDegrees())
                        < ShooterConstants.kHeadingTolerance.in(Degrees);
        Logger.recordOutput("Shooting/Ready to shoot", flywheelReady && headingReady);
        return flywheelReady && headingReady;
    }

    public Rotation2d getTargetHeading() {
        return latestParameters != null ? latestParameters.targetHeading() : getAngleToHub(new Pose2d());
    }

    /**
     * Unified command that aims the robot and spins up the shooter based on trajectory calculation.
     *
     * @param drive The drive subsystem
     * @param xSupplier Translation X
     * @param ySupplier Translation Y
     * @return Command that aims and spins up, finishing when ready to shoot
     */
    public Command aimAndSpinUp(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return Commands.parallel(
                        autoSpeedShooter(drive::getPose, drive::getChassisSpeeds),
                        DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, this::getTargetHeading))
                .until(() -> isReadyToShoot(drive.getRotation()))
                .withName("AimAndSpinUp");
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
        if ((manualShootingEnabled.get() == 1.0) || vision.getTagCount() == 0) {
            return autoSpeedShooter(drive::getPose, drive::getChassisSpeeds);
        } else {
            return fullAutoAim(drive, xSupplier, ySupplier);
        }
    }

    /** Manual override command for testing and bench mode. Doesn't run the shooter */
    public Command setFlywheelVelocityManual(AngularVelocity velocity) {
        return Commands.runOnce(() -> manualShootingVelocity = velocity);
    }

    public Command changeFlywheelVelocityManual(AngularVelocity deltaRPM) {
        return Commands.runOnce(() -> manualShootingVelocity = manualShootingVelocity.plus(deltaRPM));
    }

    public Command changeManualShootingCommand(Command command) {
        return Commands.runOnce(() -> currentShootingCommand = command);
    }

    public Supplier<Command> getCurrentShootingCommandSupplier() {
        return () -> currentShootingCommand;
    }

    public Command setManualShootingEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> {
                    manualShootingEnabled.set(enabled ? 1.0 : 0.0);
                })
                .withName("SetManualShootingEnabled:" + enabled);
    }

    public Command runToggledSpeed(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> chassisSpeeds) {
        if (manualShootingEnabled.get() == 1.0) {
            return runFlywheelVelocityManual();
        } else {
            return autoSpeedShooter(robotPose, chassisSpeeds);
        }
    }

    public Command runFlywheelVelocityManual() {
        return Commands.run(
                        () -> {
                            setFlywheelVelocity(manualShootingVelocity);
                        },
                        shooter.getLeft(),
                        shooter.getRight())
                .withName("RunFlywheelVelocityManual");
    }
}
