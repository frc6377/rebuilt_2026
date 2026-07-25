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

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FullAuto.FullAutoReal;
import frc.robot.FullAuto.FullAutoSim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathGenerator;
import frc.robot.commands.ShooterCalibrationCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.extender.ExtenderIO;
import frc.robot.subsystems.intake.extender.ExtenderIOReal;
import frc.robot.subsystems.intake.extender.ExtenderIOSim;
import frc.robot.subsystems.shooter.BaseShooterIO;
import frc.robot.subsystems.shooter.BaseShooterIOKrakenX60;
import frc.robot.subsystems.shooter.BaseShooterIOSim;
import frc.robot.subsystems.superstructure.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.GamePieceCameraSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.questnav.QuestNavIO;
import frc.robot.util.OILayer.OI;
import frc.robot.util.OILayer.OIKeyboard;
import frc.robot.util.OILayer.OIXbox;
import java.util.Objects;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    protected final Superstructure superstructure;

    private final Drive drive;

    // Jay was here and basiclly is the reason that this code works <3
    // Good job Jay! -Jackson A.

    private final Vision vision;
    protected final Intake intake;
    private final OI OIController;
    private final Indexer indexer;
    private final SwerveDriveSimulation driveSimulation;
    private final IntakeSimulation intakeSimulation;

    private GamePieceCameraSim gamePieceCamera;

    private final boolean usingController;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedDashboardChooser<DriveMode> driveModeChooser;

    public enum DriveMode {
        MANUAL,
        FULL_AUTO
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        RobotState.create();
        PathGenerator.registerTrenchZones();

        usingController = true;

        if (usingController || Constants.currentMode != Constants.Mode.SIM) {
            OIController = new OIXbox();
        } else {
            OIController = new OIKeyboard();
        }
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                if (Constants.EnabledSubsystems.kDrive) {
                    drive = new Drive(
                            new GyroIOPigeon2(),
                            new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                            new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                            new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                            new ModuleIOTalonFXReal(TunerConstants.BackRight),
                            (pose) -> {});
                } else {
                    drive = new Drive(
                            new GyroIO() {},
                            new ModuleIO() {},
                            new ModuleIO() {},
                            new ModuleIO() {},
                            new ModuleIO() {},
                            (pose) -> {});
                }
                vision = new Vision(
                        drive, new QuestNavIO() {}, new VisionIOLimelight("limelight-shooter", drive::getRotation));
                intake = new Intake(
                        Constants.EnabledSubsystems.kExtender ? new ExtenderIOReal() : new ExtenderIO() {},
                        Constants.EnabledSubsystems.kRoller
                                ? new BaseShooterIOKrakenX60(
                                        frc.robot.subsystems.intake.IntakeConstants.RollerConstants.rollerConfig)
                                : new BaseShooterIO() {});
                indexer = new Indexer(Constants.EnabledSubsystems.kIndexer ? new IndexerIOReal() : new IndexerIO() {});
                driveSimulation = null;
                intakeSimulation = null;
                gamePieceCamera = null;
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                // Default MapleSim timing is 5 subticks/period (~44ms arena step here).
                // 3 subticks ≈ 150 Hz physics — still usable, much cheaper on CPU.
                SimulatedArena.overrideSimulationTimings(Seconds.of(0.02), 3);

                driveSimulation =
                        new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3.0, 3.0, new Rotation2d()));
                intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                        "Fuel",
                        driveSimulation,
                        IntakeConstants.kIntakeExtension,
                        IntakeConstants.kIntakeWidth,
                        IntakeSimulation.IntakeSide.FRONT,
                        IntakeConstants.kIntakeCapacity);
                // Detect contacts for pickup, but don't physically collide with
                // field/obstacles.
                intakeSimulation.setSensor(true);
                intake = new Intake(
                        Constants.EnabledSubsystems.kExtender ? new ExtenderIOSim() : new ExtenderIO() {},
                        Constants.EnabledSubsystems.kRoller
                                ? new BaseShooterIOSim(
                                        frc.robot.subsystems.intake.IntakeConstants.RollerConstants.rollerConfig)
                                : new BaseShooterIO() {});
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        (pose) -> driveSimulation.setSimulationWorldPose(pose));

                drive.setPoseSupplier(driveSimulation::getSimulatedDriveTrainPose);
                PathGenerator.setPoseSupplier(driveSimulation::getSimulatedDriveTrainPose);
                vision = new Vision(
                        drive,
                        new QuestNavIO() {},
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                gamePieceCamera =
                        new GamePieceCameraSim(new Transform3d(), driveSimulation::getSimulatedDriveTrainPose);
                indexer = new Indexer(Constants.EnabledSubsystems.kIndexer ? new IndexerIOSim() : new IndexerIO() {});
                break;
            default:
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                driveSimulation = null;
                intakeSimulation = null;
                gamePieceCamera = null;
                vision = new Vision(drive, new QuestNavIO() {}, new VisionIO() {}, new VisionIO() {});
                intake = new Intake(new ExtenderIO() {}, new BaseShooterIO() {});
                indexer = new Indexer(new IndexerIO() {});
                break;
        }

        superstructure = new Superstructure(intake::isRollerRunning, vision, OIController);

        if (Constants.currentMode == Constants.Mode.SIM) {
            superstructure.configureGamePieceSimulation(driveSimulation);
        }

        intake.setRetractBlockedSupplier(this::hasHeldFuel);
        NamedCommands.registerCommand(
                "Stop shooter", superstructure.stopShooterCommand().alongWith(superstructure.stopUpgoerCommand()));
        NamedCommands.registerCommand(
                "Unjam",
                superstructure.unjamCommand().alongWith(superstructure.setFlywheelVelocityCommand(RPM.of(-1500))));
        NamedCommands.registerCommand("SpinUpHub", superstructure.setFlywheelVelocityCommand(RPM.of(2600)));
        NamedCommands.registerCommand(
                "Spin Up Shooter and Wait", superstructure.setFlywheelVelocityAndWaitCommand(RPM.of(3000)));
        NamedCommands.registerCommand(
                "AutoShootHub",
                superstructure
                        .autoSpeedShooter(drive::getPose, drive::getChassisSpeeds)
                        .until(superstructure::atTargetVelocity)
                        .andThen(Commands.parallel(
                                indexer.index(),
                                superstructure.fireCommand(),
                                intake.intakeRollerCommand(),
                                intake.siftFuelCommand())));
        NamedCommands.registerCommand(
                "AutoShoot",
                superstructure
                        .autoSpeedShooter(drive::getPose, drive::getChassisSpeeds)
                        .until(superstructure::atTargetVelocity)
                        .andThen(Commands.parallel(
                                superstructure.fireCommand(),
                                indexer.index(),
                                intake.intakeRollerCommand(),
                                intake.siftFuelCommand())));
        NamedCommands.registerCommand(
                "AutoEverything",
                Commands.sequence(
                        Commands.parallel(superstructure.autoSpeedShooter(), intake.intakeCommand())
                                .until(superstructure::atTargetVelocity),
                        Commands.parallel(intake.intakeCommand(), superstructure.fireCommand())));

        NamedCommands.registerCommand(
                "Shoot", Commands.deadline(Commands.waitSeconds(5), superstructure.fireCommand()));
        // NamedCommands.registerCommand("Intake",
        // Commands.deadline(intake.intakeCommand(), Commands.waitSeconds(6)));
        NamedCommands.registerCommand("Wait 5 seconds", Commands.waitSeconds(5));
        NamedCommands.registerCommand("Extend Intake", intake.extendIntake());
        NamedCommands.registerCommand("Intake", Commands.parallel(intake.intakeCommand(), indexer.index()));
        NamedCommands.registerCommand(
                "Index",
                Commands.runOnce(() -> indexer.setRunning(true)).withTimeout(3).andThen(indexer.stop()));
        NamedCommands.registerCommand(
                "Auto Aim", superstructure.aimAtHubWhileDriving(drive, () -> 0, () -> 0, () -> OIController.xDrive()
                        .getAsBoolean()));
        NamedCommands.registerCommand("Stop intake", intake.stopRollerCommand());
        NamedCommands.registerCommand(
                "Pathfind to Hub",
                PathGenerator.pathfindToPose(drive, new Pose2d(FieldConstants.getHubPosition(), new Rotation2d())));

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        if (Constants.currentMode == Constants.Mode.SIM) {
            autoChooser.addOption("Shooter Tuning Sim", new ShooterCalibrationCommand(superstructure, driveSimulation));
        }

        // Separate from auto: enables self-driving during teleop when Full Auto is
        // selected
        driveModeChooser = new LoggedDashboardChooser<>("Drive Mode");
        driveModeChooser.addDefaultOption("Manual", DriveMode.MANUAL);
        driveModeChooser.addOption("Full Auto", DriveMode.FULL_AUTO);
        new Trigger(() -> driveModeChooser.get() == DriveMode.FULL_AUTO)
                .and(DriverStation::isTeleopEnabled)
                .whileTrue(
                        Constants.currentMode == Constants.Mode.SIM ? new FullAutoSim(this) : new FullAutoReal(this));
        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId All",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .andThen(Commands.waitSeconds(1))
                        .andThen(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(drive.sysIdDynamic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(SignalLogger::stop));
        autoChooser.addOption(
                "Drive SysID Turning (All)",
                drive.sysIdDynamicTurning(SysIdRoutine.Direction.kForward)
                        .andThen(Commands.waitSeconds(1))
                        .andThen(drive.sysIdDynamicTurning(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(drive.sysIdQuasistaticTurning(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(drive.sysIdQuasistaticTurning(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop));
        autoChooser.addOption(
                "Left Shooter Flywheel Characterization All",
                superstructure
                        .getLeftShooter()
                        .sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .andThen(Commands.waitSeconds(1))
                        .andThen(superstructure.getLeftShooter().sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(superstructure.getLeftShooter().sysIdDynamic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(superstructure.getLeftShooter().sysIdDynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop));
        autoChooser.addOption(
                "Right Shooter Flywheel Characterization All",
                Commands.runOnce(SignalLogger::start)
                        .andThen(superstructure.getRightShooter().sysIdQuasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(superstructure.getRightShooter().sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(superstructure.getRightShooter().sysIdDynamic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(superstructure.getRightShooter().sysIdDynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop));
        autoChooser.addOption(
                "Intake Flywheel Char",
                Commands.runOnce(SignalLogger::start)
                        .andThen(intake.getRoller().sysIdQuasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(intake.getRoller().sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(intake.getRoller().sysIdDynamic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(intake.getRoller().sysIdDynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                // The lambda () -> ensures this check happens every loop
                this::driveTranslationYInput,
                this::driveTranslationXInput,
                () -> OIController.driveRotation().getAsDouble(),
                () -> OIController.xDrive().getAsBoolean()));
        // // Lock to 0° when butn is
        // OIController.driveLock0()
        // .whileTrue(DriveCommands.joystickDriveAtAngle(
        // drive,
        // () -> -OIController.driveTranslationY().getAsDouble(),
        // () -> -OIController.driveTranslationX().getAsDouble(),
        // () -> new Rotation2d()));

        // OIController.spinUpShooter()
        // .whileTrue(superstructure.autoChooseShootingCommand(
        // drive, OIController.driveTranslationX(), OIController.driveTranslationY()));

        // Manual fire (feeds piece when shooter is ready)
        // OIController.fireShooter()
        // .whileTrue(superstructure
        // .autoSpeedShooter(drive::getPose, drive::getChassisSpeeds)
        // .alongWith(superstructure.aimAtHubWhileDriving(
        // drive, OIController.driveTranslationX(), OIController.driveTranslationY()))
        // .until(superstructure::atTargetVelocity)
        // .andThen(superstructure.fireCommand())
        // .alongWith(indexer.index())
        // .alongWith(Commands.runOnce(drive::stopWithX)))
        // .onFalse(superstructure.stopUpgoerCommand().alongWith(indexer.stop()));
        Trigger shootingTrigger = OIController.fireShooter().or(OIController.shootDriver());
        Trigger stopSuperTrigger = OIController.stopShooterDriver().or(OIController.stopSuperstructure());
        OIController.spinUpShooter()
                .whileTrue(Commands.parallel(superstructure.runFlywheelVelocityManual())
                        .until(superstructure::atTargetVelocity)
                        .andThen(indexer.index())
                        .alongWith(superstructure.fireCommand()))
                .onFalse(superstructure
                        .setFlywheelVelocityManual(RPM.of(1500))
                        .andThen(superstructure.runFlywheelVelocityManual()));

        // Commands.either(Manual(), Auto(), ismanual)
        //
        shootingTrigger
                .and(OIController.manualHold().negate())
                .whileTrue(shootAutoCommand(
                        this::driveTranslationXInput, this::driveTranslationYInput, OIController.xDrive()))
                .onFalse(shootAutoStopCommand());
        shootingTrigger
                .and(OIController.manualHold())
                .whileTrue(Commands.parallel(superstructure
                        .runFlywheelVelocityManual()
                        // superstructure.aimAtHubWhileDriving(
                        // drive, OIController.driveTranslationX(),
                        // OIController.driveTranslationY()))
                        .withTimeout(0.5)
                        .until(superstructure::atTargetVelocity)
                        .andThen(Commands.runOnce(drive::stopWithX))
                        .andThen(Commands.parallel(
                                superstructure.fireCommand(), indexer.index(), intake.siftFuelCommand()))))
                .onFalse(Commands.parallel(
                                superstructure.stopUpgoerCommand(),
                                indexer.stop(),
                                superstructure.setFlywheelVelocityManual(RPM.of(1500)),
                                intake.extendIntake())
                        .andThen(superstructure.runFlywheelVelocityManual()));

        OIController.unjamShooter()
                .whileTrue(superstructure.unjamCommand().alongWith(indexer.indexReverse()))
                .onFalse(superstructure.stopUpgoerCommand().alongWith(superstructure.stopShooterCommand()));

        // Stop all components
        stopSuperTrigger.onTrue(superstructure.stopShooterCommand().alongWith(superstructure.stopUpgoerCommand()));

        OIController.autoSpeedMode()
                .onTrue(superstructure.changeManualShootingCommand(superstructure.autoChooseShootingCommand(
                        drive, this::driveTranslationXInput, this::driveTranslationYInput)));
        OIController.hubShootSpeed()
                .onTrue(superstructure
                        .setFlywheelVelocityManual(RPM.of(2600))
                        .alongWith(superstructure.setManualShootingEnabledCommand(true))
                        .andThen(superstructure.runFlywheelVelocityManual()));
        OIController.towerShootSpeed()
                .onTrue(superstructure
                        .setFlywheelVelocityManual(RPM.of(3200))
                        .alongWith(superstructure.setManualShootingEnabledCommand(true))
                        .andThen(superstructure.runFlywheelVelocityManual()));
        OIController.cornerShootSpeed()
                .onTrue(superstructure
                        .setFlywheelVelocityManual(RPM.of(3800))
                        .alongWith(superstructure.setManualShootingEnabledCommand(true))
                        .andThen(superstructure.runFlywheelVelocityManual()));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
                : () -> drive.setPose(new Pose2d(
                        drive.getPose().getTranslation(),
                        new Rotation2d(
                                DriverStation.getAlliance().get() == Alliance.Blue
                                        ? Degrees.zero()
                                        : Degrees.of(180))));
        // OIController.zeroDrivebase().onTrue(Commands.runOnce(resetGyro,
        // drive).ignoringDisable(true));
        OIController.zeroDrivebase()
                .onTrue(PathGenerator.pathfindToPose(
                        drive, new Pose2d(new Translation2d(15, 7), new Rotation2d().fromDegrees(90))));
        // OIController.zeroDrivebase()
        // .onTrue(PathGenerator.pathfindAdStar(
        // drive, new Pose2d(new Translation2d(11.5, 7.5), new
        // Rotation2d().fromDegrees(90))));

        OIController.intake()
                .and(shootingTrigger.negate())
                .whileTrue(Commands.parallel(intake.intakeRollerCommand(), indexer.index(), intake.extendIntake()))
                .onFalse(indexer.stop().alongWith(intake.idleRollerCommand()).unless(shootingTrigger::getAsBoolean));
        OIController.outtake()
                .and(shootingTrigger.negate())
                .whileTrue(intake.outtakeRollerCommand().alongWith(indexer.indexReverse()))
                .onFalse(indexer.stop().alongWith(intake.idleRollerCommand()).unless(shootingTrigger::getAsBoolean));

        OIController.toggleIntake().and(shootingTrigger.negate()).onTrue(intake.toggleIntake());
        OIController.intakeMiddle().and(shootingTrigger.negate()).onTrue(intake.goToCustomAngleOneCommand());
    }

    /**
     * s Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public Drive getDrive() {
        return drive;
    }

    /** Same command bound to the non-manual shooting trigger. */
    public Command shootAutoCommand(
            java.util.function.DoubleSupplier xSupplier,
            java.util.function.DoubleSupplier ySupplier,
            java.util.function.BooleanSupplier xModePressed) {
        return Commands.parallel(
                        superstructure
                                .aimAtHubWhileDriving(drive, xSupplier, ySupplier, xModePressed)
                                .repeatedly(),
                        Commands.sequence(
                                superstructure
                                        .autoSpeedShooter(drive::getPose, drive::getChassisSpeeds)
                                        .until(() -> superstructure.isReadyToShoot(drive.getRotation()))
                                        .withTimeout(0.7),
                                Commands.parallel(
                                        superstructure.fireCommand(),
                                        indexer.index(),
                                        intake.siftFuelCommand(),
                                        intake.intakeRollerCommand())))
                .withName("ShootAuto");
    }

    public Command shootAutoModeCommand(Drive drive1) {
        return Commands.sequence(
                        superstructure
                                .autoSpeedShooter(drive1::getPose, drive1::getChassisSpeeds)
                                .until(() -> superstructure.isReadyToShoot(drive1.getRotation()))
                                .withTimeout(0.7),
                        Commands.parallel(
                                superstructure.fireCommand(),
                                indexer.index(),
                                intake.siftFuelCommand(),
                                intake.intakeRollerCommand()))
                .withName("ShootAutoMode");
    }

    /** Same cleanup as shooting trigger onFalse. */
    public Command shootAutoStopCommand() {
        return Commands.parallel(
                        Commands.runOnce(
                                () -> superstructure.getLeftUpgoer().setVelocity(RPM.of(-200)),
                                superstructure.getLeftUpgoer()),
                        Commands.runOnce(
                                () -> superstructure.getRightUpgoer().setVelocity(RPM.of(-200)),
                                superstructure.getRightUpgoer()),
                        indexer.stop(),
                        superstructure.setFlywheelVelocityManual(RPM.of(1500)),
                        intake.extendIntake())
                .withName("Shoot Manual Stop")
                .andThen(superstructure.runFlywheelVelocityManual())
                .andThen(intake.stopRollerCommand());
    }

    public Vision getVision() {
        return vision;
    }

    public Intake getIntake() {
        return intake;
    }

    public Indexer getIndexer() {
        return indexer;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public GamePieceCameraSim getGamePieceCamera() {
        return gamePieceCamera;
    }

    public IntakeSimulation getIntakeSimulation() {
        return intakeSimulation;
    }

    public boolean hasHeldFuel() {
        if (intakeSimulation != null && intakeSimulation.getGamePiecesAmount() > 0) {
            return true;
        }
        var traj = superstructure.getGamePieceTrajectorySimulation();
        return traj != null && traj.hasBalls();
    }

    private static final Pose2d kSimStartPose = new Pose2d(3.0, 3.0, Rotation2d.kZero);
    private boolean simPoseSet = false;

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;
        SimulatedArena.getInstance().resetFieldForAuto();
        simPoseSet = false;
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        if (!simPoseSet && DriverStation.getAlliance().isPresent()) {
            drive.setPose(FieldConstants.toCurrentAlliancePose(kSimStartPose));
            simPoseSet = true;
        }

        SimulatedArena.getInstance().simulationPeriodic();
        if (intake.isRollerRunningSupplier().getAsBoolean()
                && intake.isExtendedSupplier().getAsBoolean()) {
            intakeSimulation.startIntake();
        } else {
            intakeSimulation.stopIntake();
        }

        Pose3d[] fuel = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
        Pose3d[] visibleFuel = gamePieceCamera.getVisiblePieces();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", fuel);
        Logger.recordOutput("FieldSimulation/VisibleFuel", visibleFuel);
        Logger.recordOutput("FieldSimulation/FuelCount", fuel.length);
        Logger.recordOutput("FieldSimulation/VisibleFuelCount", visibleFuel.length);
        Logger.recordOutput("FieldSimulation/Intake/IntakedGamepieces", intakeSimulation.getGamePiecesAmount());
        Logger.recordOutput(
                "Shooting/WhoWonAuton",
                Objects.equals(DriverStation.getGameSpecificMessage(), "B") ? "363AF4" : "F44336");
    }

    public Command getRobotStartPose(int cameraIndex) {
        return Commands.runOnce(() -> {
                    Pose3d cameraPose = vision.getStartingPoseFromCamera(cameraIndex);
                    if (cameraPose != null) {
                        drive.setPose(cameraPose.toPose2d());
                    }
                })
                .ignoringDisable(true);
    }

    // DriveCommands already inverts stick sign into field coords; do not negate
    // again in sim.
    private double driveTranslationXInput() {
        return OIController.driveTranslationX().getAsDouble();
    }

    private double driveTranslationYInput() {
        return OIController.driveTranslationY().getAsDouble();
    }
}
