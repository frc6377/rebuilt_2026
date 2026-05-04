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
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
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
import frc.robot.subsystems.intake.extender.ExtenderIO;
import frc.robot.subsystems.intake.extender.ExtenderIOReal;
import frc.robot.subsystems.intake.extender.ExtenderIOSim;
import frc.robot.subsystems.shooter.BaseShooterIO;
import frc.robot.subsystems.shooter.BaseShooterIOKrakenX60;
import frc.robot.subsystems.shooter.BaseShooterIOSim;
import frc.robot.subsystems.superstructure.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.questnav.QuestNavIO;
import frc.robot.util.OILayer.OI;
import frc.robot.util.OILayer.OIKeyboard;
import frc.robot.util.OILayer.OIXbox;
import java.util.Objects;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    protected final @NotNull Superstructure superstructure;

    private final @NotNull Drive drive;

    // Jay was here and basiclly is the reason that this code works <3
    // Good job Jay! -Jackson A.

    private final @NotNull Vision vision;
    protected final @NotNull Intake intake;
    private final @NotNull OI OIController;
    private final @NotNull Indexer indexer;
    private final @Nullable SwerveDriveSimulation driveSimulation; // Only used in simulation, but declared here for easy
    // access by subsystems that need it

    // Dashboard inputs
    private final @NotNull LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        RobotState.create();

        boolean usingController = false;

        if (usingController || Constants.Mode.SIM != Constants.currentMode) {
            this.OIController = new OIXbox();
        } else {
            this.OIController = new OIKeyboard();
        }
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                if (Constants.EnabledSubsystems.kDrive) {
                    this.drive = new Drive(
                            new GyroIOPigeon2(),
                            new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                            new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                            new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                            new ModuleIOTalonFXReal(TunerConstants.BackRight),
                            (pose) -> {});
                } else {
                    this.drive = new Drive(
                            new GyroIO() {},
                            new ModuleIO() {},
                            new ModuleIO() {},
                            new ModuleIO() {},
                            new ModuleIO() {},
                            (pose) -> {});
                }
                this.vision = new Vision(
                        this.drive, new QuestNavIO() {}, new VisionIOLimelight("limelight-shooter", this.drive::getRotation));
                this.intake = new Intake(
                        Constants.EnabledSubsystems.kExtender ? new ExtenderIOReal() : new ExtenderIO() {},
                        Constants.EnabledSubsystems.kRoller
                                ? new BaseShooterIOKrakenX60(
                                        frc.robot.subsystems.intake.IntakeConstants.RollerConstants.rollerConfig)
                                : new BaseShooterIO() {});
                this.indexer = new Indexer(Constants.EnabledSubsystems.kIndexer ? new IndexerIOReal() : new IndexerIO() {});
                this.driveSimulation = null;
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                this.driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                this.intake = new Intake(
                        Constants.EnabledSubsystems.kExtender ? new ExtenderIOSim() : new ExtenderIO() {},
                        Constants.EnabledSubsystems.kRoller
                                ? new BaseShooterIOSim(
                                        frc.robot.subsystems.intake.IntakeConstants.RollerConstants.rollerConfig)
                                : new BaseShooterIO() {});
                SimulatedArena.getInstance().addDriveTrainSimulation(this.driveSimulation);
                this.drive = new Drive(
                        new GyroIOSim(this.driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, this.driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, this.driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, this.driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, this.driveSimulation.getModules()[3]),
                        (pose) -> this.driveSimulation.setSimulationWorldPose(pose));
                this.vision = new Vision(
                        this.drive,
                        new QuestNavIO() {},
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, this.driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, this.driveSimulation::getSimulatedDriveTrainPose));
                this.indexer = new Indexer(Constants.EnabledSubsystems.kIndexer ? new IndexerIOSim() : new IndexerIO() {});
                break;
            default:
                this.drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                this.driveSimulation = null;
                this.vision = new Vision(this.drive, new QuestNavIO() {}, new VisionIO() {}, new VisionIO() {});
                this.intake = new Intake(new ExtenderIO() {}, new BaseShooterIO() {});
                this.indexer = new Indexer(new IndexerIO() {});
                break;
        }

        this.superstructure = new Superstructure(this.intake::isRollerRunning, this.vision, this.OIController);

        if (Constants.Mode.SIM == Constants.currentMode) {
            this.superstructure.configureGamePieceSimulation(this.driveSimulation);
        }
        NamedCommands.registerCommand(
                "Stop shooter", this.superstructure.stopShooterCommand().alongWith(this.superstructure.stopUpgoerCommand()));
        NamedCommands.registerCommand(
                "Unjam",
                this.superstructure.unjamCommand().alongWith(this.superstructure.setFlywheelVelocityCommand(RPM.of(-1500))));
        NamedCommands.registerCommand("SpinUpHub", this.superstructure.setFlywheelVelocityCommand(RPM.of(2600)));
        NamedCommands.registerCommand(
                "Spin Up Shooter and Wait", this.superstructure.setFlywheelVelocityAndWaitCommand(RPM.of(3000)));
        NamedCommands.registerCommand(
                "AutoShootHub",
                this.superstructure
                        .autoSpeedShooter(this.drive::getPose, this.drive::getChassisSpeeds)
                        .until(this.superstructure::atTargetVelocity)
                        .andThen(Commands.parallel(
                                this.indexer.index(),
                                this.superstructure.fireCommand(),
                                this.intake.intakeRollerCommand(),
                                this.intake.siftFuelCommand())));
        NamedCommands.registerCommand(
                "AutoShoot",
                this.superstructure
                        .autoSpeedShooter(this.drive::getPose, this.drive::getChassisSpeeds)
                        .until(this.superstructure::atTargetVelocity)
                        .andThen(Commands.parallel(
                                this.superstructure.fireCommand(),
                                this.indexer.index(),
                                this.intake.intakeRollerCommand(),
                                this.intake.siftFuelCommand())));
        NamedCommands.registerCommand(
                "AutoEverything",
                Commands.sequence(
                        Commands.parallel(this.superstructure.autoSpeedShooter(), this.intake.intakeCommand())
                                .until(this.superstructure::atTargetVelocity),
                        Commands.parallel(this.intake.intakeCommand(), this.superstructure.fireCommand())));

        NamedCommands.registerCommand(
                "Shoot", Commands.deadline(Commands.waitSeconds(5), this.superstructure.fireCommand()));
        // NamedCommands.registerCommand("Intake",
        // Commands.deadline(intake.intakeCommand(), Commands.waitSeconds(6)));
        NamedCommands.registerCommand("Wait 5 seconds", Commands.waitSeconds(5));
        NamedCommands.registerCommand("Extend Intake", this.intake.extendIntake());
        NamedCommands.registerCommand("Intake", Commands.parallel(this.intake.intakeCommand(), this.indexer.index()));
        NamedCommands.registerCommand(
                "Index",
                Commands.runOnce(() -> this.indexer.setRunning(true)).withTimeout(3).andThen(this.indexer.stop()));
        NamedCommands.registerCommand(
                "Auto Aim", this.superstructure.aimAtHubWhileDriving(this.drive, () -> 0, () -> 0, () -> this.OIController.xDrive()
                        .getAsBoolean()));
        NamedCommands.registerCommand("Stop intake", this.intake.stopRollerCommand());

        // Set up auto routines
        this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        if (Constants.Mode.SIM == Constants.currentMode) {
            this.autoChooser.addOption("Shooter Tuning Sim", new ShooterCalibrationCommand(this.superstructure, this.driveSimulation));
        }
        // Set up SysId routines
        this.autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(this.drive));
        this.autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(this.drive));
        this.autoChooser.addOption(
                "Drive SysId All",
                this.drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .andThen(Commands.waitSeconds(1))
                        .andThen(this.drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(this.drive.sysIdDynamic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(this.drive.sysIdDynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(SignalLogger::stop));
        this.autoChooser.addOption(
                "Drive SysID Turning (All)",
                this.drive.sysIdDynamicTurning(SysIdRoutine.Direction.kForward)
                        .andThen(Commands.waitSeconds(1))
                        .andThen(this.drive.sysIdDynamicTurning(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(this.drive.sysIdQuasistaticTurning(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(this.drive.sysIdQuasistaticTurning(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop));
        this.autoChooser.addOption(
                "Left Shooter Flywheel Characterization All",
                this.superstructure
                        .getLeftShooter()
                        .sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .andThen(Commands.waitSeconds(1))
                        .andThen(this.superstructure.getLeftShooter().sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(this.superstructure.getLeftShooter().sysIdDynamic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(1))
                        .andThen(this.superstructure.getLeftShooter().sysIdDynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop));
        this.autoChooser.addOption(
                "Right Shooter Flywheel Characterization All",
                Commands.runOnce(SignalLogger::start)
                        .andThen(this.superstructure.getRightShooter().sysIdQuasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(this.superstructure.getRightShooter().sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(this.superstructure.getRightShooter().sysIdDynamic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(this.superstructure.getRightShooter().sysIdDynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop));
        this.autoChooser.addOption(
                "Intake Flywheel Char",
                Commands.runOnce(SignalLogger::start)
                        .andThen(this.intake.getRoller().sysIdQuasistatic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(this.intake.getRoller().sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(this.intake.getRoller().sysIdDynamic(SysIdRoutine.Direction.kForward))
                        .andThen(Commands.waitSeconds(5))
                        .andThen(this.intake.getRoller().sysIdDynamic(SysIdRoutine.Direction.kReverse))
                        .andThen(SignalLogger::stop));

        // Configure the button bindings
        this.configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        this.drive.setDefaultCommand(DriveCommands.joystickDrive(
                this.drive,
                // The lambda () -> ensures this check happens every loop
                () -> this.OIController.driveTranslationY().getAsDouble(),
                () -> this.OIController.driveTranslationX().getAsDouble(),
                () -> this.OIController.driveRotation().getAsDouble(),
                () -> this.OIController.xDrive().getAsBoolean()));
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
        Trigger shootingTrigger = this.OIController.fireShooter().or(this.OIController.shootDriver());
        Trigger stopSuperTrigger = this.OIController.stopShooterDriver().or(this.OIController.stopSuperstructure());
        this.OIController.spinUpShooter()
                .whileTrue(Commands.parallel(this.superstructure.runFlywheelVelocityManual())
                        .until(this.superstructure::atTargetVelocity)
                        .andThen(this.indexer.index())
                        .alongWith(this.superstructure.fireCommand()))
                .onFalse(this.superstructure
                        .setFlywheelVelocityManual(RPM.of(1500))
                        .andThen(this.superstructure.runFlywheelVelocityManual()));

        // Commands.either(Manual(), Auto(), ismanual)
        //
        shootingTrigger
                .and(this.OIController.manualHold().negate())
                .whileTrue(Commands.parallel(
                                this.superstructure
                                        .aimAtHubWhileDriving(
                                                this.drive,
                                                this.OIController.driveTranslationX(),
                                                this.OIController.driveTranslationY(),
                                                this.OIController.xDrive())
                                        .repeatedly(),
                                Commands.sequence(
                                        this.superstructure
                                                .autoSpeedShooter(this.drive::getPose, this.drive::getChassisSpeeds)
                                                .until(() -> this.superstructure.isReadyToShoot(this.drive.getRotation()))
                                                .withTimeout(0.7),
                                        Commands.parallel(
                                                this.superstructure.fireCommand(),
                                                this.indexer.index(),
                                                this.intake.siftFuelCommand(),
                                                this.intake.intakeRollerCommand())))
                        .withName("ShootAuto"))
                .onFalse(Commands.parallel(
                                Commands.runOnce(
                                        () -> this.superstructure.getLeftUpgoer().setVelocity(RPM.of(-200)),
                                        this.superstructure.getLeftUpgoer()),
                                Commands.runOnce(
                                        () -> this.superstructure.getRightUpgoer().setVelocity(RPM.of(-200)),
                                        this.superstructure.getRightUpgoer()),
                                this.indexer.stop(),
                                this.superstructure.setFlywheelVelocityManual(RPM.of(1500)),
                                this.intake.extendIntake())
                        .withName("Shoot Manual Stop")
                        .andThen(this.superstructure.runFlywheelVelocityManual())
                        .andThen(this.intake.stopRollerCommand()));
        shootingTrigger
                .and(this.OIController.manualHold())
                .whileTrue(Commands.parallel(this.superstructure
                        .runFlywheelVelocityManual()
                        // superstructure.aimAtHubWhileDriving(
                        // drive, OIController.driveTranslationX(),
                        // OIController.driveTranslationY()))
                        .withTimeout(0.5)
                        .until(this.superstructure::atTargetVelocity)
                        .andThen(Commands.runOnce(this.drive::stopWithX))
                        .andThen(Commands.parallel(
                                this.superstructure.fireCommand(), this.indexer.index(), this.intake.siftFuelCommand()))))
                .onFalse(Commands.parallel(
                                this.superstructure.stopUpgoerCommand(),
                                this.indexer.stop(),
                                this.superstructure.setFlywheelVelocityManual(RPM.of(1500)),
                                this.intake.extendIntake())
                        .andThen(this.superstructure.runFlywheelVelocityManual()));

        this.OIController.unjamShooter()
                .whileTrue(this.superstructure.unjamCommand().alongWith(this.indexer.indexReverse()))
                .onFalse(this.superstructure.stopUpgoerCommand().alongWith(this.superstructure.stopShooterCommand()));

        // Stop all components
        stopSuperTrigger.onTrue(this.superstructure.stopShooterCommand().alongWith(this.superstructure.stopUpgoerCommand()));

        this.OIController.autoSpeedMode()
                .onTrue(this.superstructure.changeManualShootingCommand(this.superstructure.autoChooseShootingCommand(
                        this.drive, this.OIController.driveTranslationX(), this.OIController.driveTranslationY())));
        this.OIController.hubShootSpeed()
                .onTrue(this.superstructure
                        .setFlywheelVelocityManual(RPM.of(2600))
                        .alongWith(this.superstructure.setManualShootingEnabledCommand(true))
                        .andThen(this.superstructure.runFlywheelVelocityManual()));
        this.OIController.towerShootSpeed()
                .onTrue(this.superstructure
                        .setFlywheelVelocityManual(RPM.of(3200))
                        .alongWith(this.superstructure.setManualShootingEnabledCommand(true))
                        .andThen(this.superstructure.runFlywheelVelocityManual()));
        this.OIController.cornerShootSpeed()
                .onTrue(this.superstructure
                        .setFlywheelVelocityManual(RPM.of(3800))
                        .alongWith(this.superstructure.setManualShootingEnabledCommand(true))
                        .andThen(this.superstructure.runFlywheelVelocityManual()));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.Mode.SIM == Constants.currentMode
                ? () -> this.drive.setPose(this.driveSimulation.getSimulatedDriveTrainPose())
                : () -> this.drive.setPose(new Pose2d(
                this.drive.getPose().getTranslation(),
                        new Rotation2d(
                                Alliance.Blue == DriverStation.getAlliance().get()
                                        ? Degrees.zero()
                                        : Degrees.of(180))));
        this.OIController.zeroDrivebase().onTrue(Commands.runOnce(resetGyro, this.drive).ignoringDisable(true));
        //        OIController.manualHold().onTrue(Commands.runOnce(drive::stopWithX, drive));
        // OIController.start().onTrue(Commands.runOnce(resetGyro,
        // drive).ignoringDisable(true));

        this.OIController.intake()
                .and(shootingTrigger.negate())
                .whileTrue(this.intake.intakeRollerCommand().alongWith(this.indexer.index()))
                .onFalse(this.indexer.stop().alongWith(this.intake.idleRollerCommand()).unless(shootingTrigger::getAsBoolean));
        this.OIController.outtake()
                .and(shootingTrigger.negate())
                .whileTrue(this.intake.outtakeRollerCommand().alongWith(this.indexer.indexReverse()))
                .onFalse(this.indexer.stop().alongWith(this.intake.idleRollerCommand()).unless(shootingTrigger::getAsBoolean));

        this.OIController.toggleIntake().and(shootingTrigger.negate()).onTrue(this.intake.toggleIntake());
        this.OIController.intakeMiddle().and(shootingTrigger.negate()).onTrue(this.intake.goToCustomAngleOneCommand());
    }

    /**
     * s Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return this.autoChooser.get();
    }

    public void resetSimulation() {
        if (Constants.Mode.SIM != Constants.currentMode || null == this.driveSimulation) return;

        this.driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    }

    public void resetSimulationField() {
        if (Constants.Mode.SIM != Constants.currentMode) return;

        this.driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.Mode.SIM != Constants.currentMode) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", this.driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        Logger.recordOutput(
                "Shooting/WhoWonAuton",
                Objects.equals(DriverStation.getGameSpecificMessage(), "B") ? "363AF4" : "F44336");
    }

    public Command getRobotStartPose(int cameraIndex) {
        return Commands.runOnce(() -> {
                    Pose3d cameraPose = this.vision.getStartingPoseFromCamera(cameraIndex);
                    if (null != cameraPose) {
                        this.drive.setPose(cameraPose.toPose2d());
                    }
                })
                .ignoringDisable(true);
    }
}
