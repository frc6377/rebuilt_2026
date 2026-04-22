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
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
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

    private final @NotNull Shooter shooter;
    private final @NotNull Upgoer leftUpgoer;
    private final @NotNull Upgoer rightUpgoer;
    private final @NotNull Indexer indexer;
    private final Vision vision;
    private final OI oi;
    private final RobotState robotState;
    private GamePieceTrajectorySimulation gamePieceTrajectorySimulation;
    private AngularVelocity manualShootingVelocity = RPM.of(ShooterConstants.kManualShootingSpeedRPM);
    private Command currentShootingCommand;
    private TrajectoryBall.@Nullable ShootingParameters latestParameters = null;

    /** Creates the superstructure and selects IO implementations by mode. */
    public Superstructure(BooleanSupplier isIntaking, Vision vision, OI oi) {
        RobotState createdState = RobotState.getInstance();
        if (null == createdState) {
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
                "Shooting/DistanceToHub", round(this.vision.getHubDistanceMeasure().in(Meters) * 100.0) / 100.0);
        if ((4 < FieldConstants.getTimeUntilHubStateChange()
                        && 7 >= FieldConstants.getTimeUntilHubStateChange()
                        && DriverStation.isTeleopEnabled())
                || 10 >= DriverStation.getMatchTime() && DriverStation.isTeleopEnabled()) {
            this.oi.setRumble(1, 1);
        } else {
            this.oi.setRumble(0, 0);
        }

        if (null == gamePieceTrajectorySimulation) {
            return;
        }

        int desiredCount = this.robotState.getSimGamePieceCount();
        if (this.gamePieceTrajectorySimulation.getBallsInHopper() != desiredCount) {
            this.gamePieceTrajectorySimulation.setBallsInHopper(desiredCount);
        }

        this.gamePieceTrajectorySimulation.updateAutoFire();
        this.robotState.setSimGamePieceCount(this.gamePieceTrajectorySimulation.getBallsInHopper());
    }

    /** Configure the game piece trajectory simulation (SIM mode only). */
    public void configureGamePieceSimulation(@Nullable SwerveDriveSimulation driveSimulation) {
        if (Constants.Mode.SIM != Constants.currentMode || null == driveSimulation) {
            return;
        }

        this.gamePieceTrajectorySimulation = new GamePieceTrajectorySimulation(
                driveSimulation, () -> this.getAverageFlywheelVelocity().in(RPM));
        this.robotState.setSimGamePieceCount(this.gamePieceTrajectorySimulation.getBallsInHopper());
    }

    public GamePieceTrajectorySimulation getGamePieceTrajectorySimulation() {
        return this.gamePieceTrajectorySimulation;
    }

    public boolean hasGamePieceTrajectorySimulation() {
        return null != gamePieceTrajectorySimulation;
    }

    public Command simAutoFireHoldCommand(BooleanSupplier indexerRunningSupplier) {
        if (null == gamePieceTrajectorySimulation) {
            return Commands.none();
        }

        return Commands.startEnd(() -> this.gamePieceTrajectorySimulation.enableAutoFire(indexerRunningSupplier), () -> {
                    this.gamePieceTrajectorySimulation.setAutoFireEnabled(false);
                    this.gamePieceTrajectorySimulation.setIndexerRunningSupplier(() -> false);
                })
                .withName("SimAutoFireHold");
    }

    public boolean simShouldIndexerRun() {
        return null != gamePieceTrajectorySimulation && this.gamePieceTrajectorySimulation.shouldIndexerRun();
    }

    public Command simLaunchGamePieceCommand() {
        if (null == gamePieceTrajectorySimulation) {
            return Commands.none();
        }

        return Commands.runOnce(() -> SimulatedArena.getInstance()
                        .addGamePieceProjectile(this.gamePieceTrajectorySimulation.launchGamePiece()))
                .withName("SimLaunchGamePiece");
    }

    public Command simAddBallsCommand(int count) {
        if (null == gamePieceTrajectorySimulation) {
            return Commands.none();
        }

        return Commands.runOnce(() -> this.gamePieceTrajectorySimulation.addBalls(count))
                .withName("SimAddBalls");
    }

    public Command simSetAutoFireEnabledCommand(boolean enabled) {
        if (null == gamePieceTrajectorySimulation) {
            return Commands.none();
        }

        return Commands.runOnce(() -> this.gamePieceTrajectorySimulation.setAutoFireEnabled(enabled))
                .withName("SimAutoFireEnabled:" + enabled);
    }

    public @Nullable Command createShooterCalibrationCommand(
            @Nullable SwerveDriveSimulation driveSimulation, Consumer<Pose2d> poseResetter) {
        if (null == gamePieceTrajectorySimulation || null == driveSimulation) {
            return null;
        }

        return new ShooterCalibrationCommand(
                this.shooter.getLeft(), this.gamePieceTrajectorySimulation, driveSimulation, poseResetter);
    }

    public BaseShooter getLeftShooter() {
        return this.shooter.getLeft();
    }

    public BaseShooter getRightShooter() {
        return this.shooter.getRight();
    }

    public Upgoer getLeftUpgoer() {
        return this.leftUpgoer;
    }

    public Upgoer getRightUpgoer() {
        return this.rightUpgoer;
    }

    public @NotNull Angle getHoodAngle() {
        return ShooterConstants.kFixedHoodAngle;
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        this.shooter.setFlywheelVelocity(velocity);
    }

    public void setUpgoerVelocity(@NotNull AngularVelocity velocity) {
        this.leftUpgoer.setVelocity(velocity);
        this.rightUpgoer.setVelocity(velocity);
    }

    public AngularVelocity getLeftFlywheelVelocity() {
        return this.shooter.getLeft().getFlywheelVelocity();
    }

    public AngularVelocity getRightFlywheelVelocity() {
        return this.shooter.getRight().getFlywheelVelocity();
    }

    public @NotNull AngularVelocity getAverageFlywheelVelocity() {
        double rpm =
                (this.getLeftFlywheelVelocity().in(RPM) + this.getRightFlywheelVelocity().in(RPM)) / 2.0;
        return RPM.of(rpm);
    }

    public void stopShooter() {
        this.shooter.stop();
    }

    public void stopUpgoer() {
        this.leftUpgoer.stop();
        this.rightUpgoer.stop();
    }

    public Command stopShooterCommand() {
        return this.shooter.stopCommand();
    }

    public Command stopUpgoerCommand() {
        return this.leftUpgoer.stopCommand().alongWith(this.rightUpgoer.stopCommand());
    }

    /** Calculate the distance from the robot to the hub. */
    public @NotNull Distance getDistanceToHub(@NotNull Pose2d robotPose) {
        return Meters.of(robotPose.getTranslation().getDistance(FieldConstants.getHubPosition()));
    }

    /** Calculate the angle from the robot to the hub. */
    public @NotNull Rotation2d getAngleToHub(@NotNull Pose2d robotPose) {
        Translation2d toHub = FieldConstants.getHubPosition().minus(robotPose.getTranslation());
        return new Rotation2d(toHub.getX(), toHub.getY());
    }

    /** Calculate the angle from the robot to the alliance wall center. */
    public @NotNull Rotation2d getAngleToAllianceWall(@NotNull Pose2d robotPose) {
        boolean isRed = Alliance.Red == DriverStation.getAlliance().orElse(Alliance.Blue);
        double targetX = isRed ? FieldConstants.fieldLength : 0.0;
        double targetY = FieldConstants.fieldWidth / 2.0;
        Translation2d target = new Translation2d(targetX, targetY);
        Translation2d toTarget = target.minus(robotPose.getTranslation());
        return new Rotation2d(toTarget.getX(), toTarget.getY());
    }

    public boolean isInShootingZone(@NotNull Pose2d robotPose) {
        double fieldLengthMeters = FieldConstants.fieldLength;
        double xMeters = robotPose.getTranslation().getX();
        boolean isRed = Alliance.Red == DriverStation.getAlliance().orElse(Alliance.Blue);
        double distanceFromOwnWall = isRed ? fieldLengthMeters - xMeters : xMeters;
        // Can shoot from own half of the field
        return distanceFromOwnWall <= fieldLengthMeters / 2.0;
    }

    private ShooterConstants.CalculationMode getCalculationModeFromDashboard() {
        int modeIndex = (int) Math.round(calculationMode.get());
        ShooterConstants.CalculationMode[] modes = ShooterConstants.CalculationMode.values();
        if (0 > modeIndex || modeIndex >= modes.length) {
            return ShooterConstants.kDefaultCalculationMode;
        }
        return modes[modeIndex];
    }

    /** Command that continuously updates flywheel speed based on distance to hub. */
    public Command autoSpeedShooter(@NotNull Supplier<Pose2d> poseSupplier, @NotNull Supplier<ChassisSpeeds> velocitySupplier) {
        return Commands.run(
                        () -> {
                            Pose2d robotPose;
                            ChassisSpeeds robotSpeeds;
                            if (0.5 < benchModeEnabled.get()) {
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
                                    this.getDistanceToHub(robotPose).in(Meters);
                            this.latestParameters = TrajectoryBall.calculate(
                                    this.getCalculationModeFromDashboard(),
                                    robotPose,
                                    robotSpeeds,
                                    Feet.of(maxHeightFeet.get()),
                                    Feet.of(targetHeightFeet.get()),
                                    rpmMultiplier.get(),
                                    ShooterConstants.kSotfEnabled);

                            Logger.recordOutput("Shooting/DistanceSource", "PoseEstimate");
                            Logger.recordOutput("Shooting/OdometryHubDistanceM", this.getCalculationModeFromDashboard());
                            Logger.recordOutput(
                                    "Shooting/TargetHeadingDeg",
                                    this.latestParameters.targetHeading().getDegrees());
                            Logger.recordOutput("Shooting/CalculationMode", calculationMode.get());

                            boolean inZone = this.isInShootingZone(robotPose);
                            Logger.recordOutput("Shooting/InShootingZone", inZone);

                            if (inZone) {
                                Logger.recordOutput("Shooting/DistanceToHub", hubDistanceMeters);
                                Logger.recordOutput(
                                        "Shooting/CalculatedRPM",
                                        this.latestParameters.flywheelVelocity().in(RPM));

                                this.setFlywheelVelocity(this.latestParameters.flywheelVelocity());
                            } else {
                                this.setFlywheelVelocity(this.manualShootingVelocity);
                            }
                        },
                        this.shooter.getLeft(),
                        this.shooter.getRight())
                .withName("AutoAimShooter");
    }

    public Command autoSpeedShooter(@NotNull Supplier<Pose2d> poseSupplier) {
        return this.autoSpeedShooter(poseSupplier, ChassisSpeeds::new);
    }

    public Command autoSpeedShooter() {
        return this.autoSpeedShooter(Pose2d::new, ChassisSpeeds::new);
    }
    /** Command that aims the robot at the hub while driving. */
    public Command aimAtHubWhileDriving(
            @NotNull Drive drive, @NotNull DoubleSupplier xSupplier, @NotNull DoubleSupplier ySupplier, @NotNull BooleanSupplier xModePressed) {
        return DriveCommands.joystickDriveAtAngle(
                        drive, xSupplier, ySupplier, () -> this.latestParameters.targetHeading(), xModePressed)
                .withName("AimAtHub");
    }

    /** Command that fires the shooter (feeds both upgoers). */
    public Command fireCommand() {
        return Commands.run(
                        () -> {
                            this.leftUpgoer.setVelocity(UpgoerConstants.defaultFeedVelocity);
                            this.rightUpgoer.setVelocity(UpgoerConstants.defaultFeedVelocity);
                        },
                        this.leftUpgoer,
                        this.rightUpgoer)
                .withName("SuperstructureFire");
    }

    public Command unjamCommand() {
        return Commands.run(
                        () -> {
                            this.leftUpgoer.setVelocity(UpgoerConstants.defaultUnjamVelocity);
                            this.rightUpgoer.setVelocity(UpgoerConstants.defaultUnjamVelocity);
                        },
                        this.leftUpgoer,
                        this.rightUpgoer)
                .withName("SuperstructureUnjam");
    }

    /** Full auto-aim command: aims robot at hub AND sets flywheel automatically. */
    public Command fullAutoAim(@NotNull Drive drive, @NotNull DoubleSupplier xSupplier, @NotNull DoubleSupplier ySupplier) {
        return this.aimAtHubWhileDriving(drive, xSupplier, ySupplier, () -> false)
                .alongWith(this.autoSpeedShooter(drive::getPose, drive::getChassisSpeeds))
                .withName("FullAutoAim")
                .beforeStarting(() -> this.robotState.setMode(RobotState.Mode.SHOOTING))
                .finallyDo(() -> this.robotState.setMode(RobotState.Mode.IDLE));
    }

    public Command spinUpShooterCommand(@NotNull Drive drive, @NotNull DoubleSupplier xSupplier, @NotNull DoubleSupplier ySupplier) {
        Supplier<Rotation2d> angleSupplier = () -> {
            Pose2d pose = drive.getPose();
            if (this.isInShootingZone(pose)) {
                return this.getAngleToHub(pose);
            }
            // On opponent's side —> shuttle back towards own wall
            return this.getAngleToAllianceWall(pose);
        };

        return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, angleSupplier)
                .alongWith(this.autoSpeedShooter(drive::getPose, drive::getChassisSpeeds))
                .withName("SpinUpShooter");
    }

    public boolean atTargetVelocity() {
        Logger.recordOutput(
                "Shooting/Ready to shoot",
                this.shooter.getLeft().atTargetVelocity() && this.shooter.getRight().atTargetVelocity());
        return this.shooter.getLeft().atTargetVelocity() && this.shooter.getRight().atTargetVelocity();
    }

    public boolean isReadyToShoot(@NotNull Rotation2d currentHeading) {
        if (null == latestParameters) return this.atTargetVelocity();

        boolean flywheelReady = this.atTargetVelocity(); // atTargetVelocity();
        boolean headingReady =
                Math.abs(currentHeading.minus(this.latestParameters.targetHeading()).getDegrees())
                        < ShooterConstants.kHeadingTolerance.in(Degrees);
        Logger.recordOutput("Shooting/Ready to shoot", flywheelReady && headingReady);
        return flywheelReady && headingReady;
    }

    public Rotation2d getTargetHeading() {
        return null != latestParameters ? this.latestParameters.targetHeading() : this.getAngleToHub(new Pose2d());
    }

    /**
     * Unified command that aims the robot and spins up the shooter based on trajectory calculation.
     *
     * @param drive The drive subsystem
     * @param xSupplier Translation X
     * @param ySupplier Translation Y
     * @return Command that aims and spins up, finishing when ready to shoot
     */
    public Command aimAndSpinUp(@NotNull Drive drive, @NotNull DoubleSupplier xSupplier, @NotNull DoubleSupplier ySupplier) {
        return Commands.parallel(
                        this.autoSpeedShooter(drive::getPose, drive::getChassisSpeeds),
                        DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, this::getTargetHeading))
                .until(() -> this.isReadyToShoot(drive.getRotation()))
                .withName("AimAndSpinUp");
    }

    public Command setFlywheelVelocityCommand(@NotNull AngularVelocity velocity) {
        return Commands.runOnce(() -> this.setFlywheelVelocity(velocity), this.shooter.getLeft(), this.shooter.getRight())
                .withName("SetFlywheelVelocity:" + velocity.in(RPM) + "RPM");
    }

    public Command setFlywheelVelocityCommand(@NotNull Supplier<AngularVelocity> velocitySupplier) {
        return Commands.run(() -> this.setFlywheelVelocity(velocitySupplier.get()), this.shooter.getLeft(), this.shooter.getRight())
                .withName("SetFlywheelVelocity");
    }

    public Command setFlywheelVelocityAndWaitCommand(@NotNull AngularVelocity velocity) {
        return this.setFlywheelVelocityCommand(velocity).until(this::atTargetVelocity);
    }

    public Command autoChooseShootingCommand(@NotNull Drive drive, @NotNull DoubleSupplier xSupplier, @NotNull DoubleSupplier ySupplier) {
        if ((1.0 == manualShootingEnabled.get()) || 0 == vision.getTagCount()) {
            return this.autoSpeedShooter(drive::getPose, drive::getChassisSpeeds);
        } else {
            return this.fullAutoAim(drive, xSupplier, ySupplier);
        }
    }

    /** Manual override command for testing and bench mode. Doesn't run the shooter */
    public @NotNull Command setFlywheelVelocityManual(AngularVelocity velocity) {
        return Commands.runOnce(() -> this.manualShootingVelocity = velocity);
    }

    public @NotNull Command changeFlywheelVelocityManual(AngularVelocity deltaRPM) {
        return Commands.runOnce(() -> this.manualShootingVelocity = this.manualShootingVelocity.plus(deltaRPM));
    }

    public @NotNull Command changeManualShootingCommand(Command command) {
        return Commands.runOnce(() -> this.currentShootingCommand = command);
    }

    public @NotNull Supplier<Command> getCurrentShootingCommandSupplier() {
        return () -> this.currentShootingCommand;
    }

    public Command setManualShootingEnabledCommand(boolean enabled) {
        return Commands.runOnce(() -> {
                    manualShootingEnabled.set(enabled ? 1.0 : 0.0);
                })
                .withName("SetManualShootingEnabled:" + enabled);
    }

    public Command runToggledSpeed(@NotNull Supplier<Pose2d> robotPose, @NotNull Supplier<ChassisSpeeds> chassisSpeeds) {
        if (1.0 == manualShootingEnabled.get()) {
            return this.runFlywheelVelocityManual();
        } else {
            return this.autoSpeedShooter(robotPose, chassisSpeeds);
        }
    }

    public Command runFlywheelVelocityManual() {
        return Commands.run(
                        () -> {
                            this.setFlywheelVelocity(this.manualShootingVelocity);
                        },
                        this.shooter.getLeft(),
                        this.shooter.getRight())
                .withName("RunFlywheelVelocityManual");
    }
}
