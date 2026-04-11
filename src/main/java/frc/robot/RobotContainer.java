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

    private final Vision vision;
    protected final Intake intake;
    private final OI OIController;
    private final Indexer indexer;
    private final SwerveDriveSimulation driveSimulation; // Only used in simulation, but declared here for easy
    // access by subsystems that need it

    private final boolean usingController;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        RobotState.create();

        usingController = false;

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
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
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
                vision = new Vision(
                        drive,
                        new QuestNavIO() {},
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
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
                vision = new Vision(drive, new QuestNavIO() {}, new VisionIO() {}, new VisionIO() {});
                intake = new Intake(new ExtenderIO() {}, new BaseShooterIO() {});
                indexer = new Indexer(new IndexerIO() {});
                break;
        }

        superstructure = new Superstructure(intake::isRollerRunning, vision, OIController);

        if (Constants.currentMode == Constants.Mode.SIM) {
            superstructure.configureGamePieceSimulation(driveSimulation);
        }
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

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        if (Constants.currentMode == Constants.Mode.SIM) {
            autoChooser.addOption("Shooter Tuning Sim", new ShooterCalibrationCommand(superstructure, driveSimulation));
        }
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
        SignalLogger.setPath("Media/sda1/logs/one/");
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
                () -> OIController.driveTranslationY().getAsDouble(),
                () -> OIController.driveTranslationX().getAsDouble(),
                () -> OIController.driveRotation().getAsDouble()));
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
                .whileTrue(Commands.parallel(
                                superstructure.aimAtHubWhileDriving(
                                        drive,
                                        OIController.driveTranslationX(),
                                        OIController.driveTranslationY(),
                                        OIController.xDrive()),
                                Commands.parallel(
                                                superstructure
                                                        .autoSpeedShooter(drive::getPose, drive::getChassisSpeeds)
                                                        .until(() ->
                                                                superstructure.isReadyToShoot(drive.getRotation())),
                                                Commands.waitSeconds(0.5))
                                        .andThen(Commands.runOnce(drive::stopWithX))
                                        .andThen(Commands.parallel(
                                                superstructure.fireCommand(),
                                                indexer.index(),
                                                intake.siftFuelCommand(),
                                                intake.intakeRollerCommand(),
                                                Commands.run(drive::stopWithX))))
                        .withName("ShootManual"))
                .onFalse(Commands.parallel(
                                superstructure.stopUpgoerCommand(),
                                indexer.stop(),
                                superstructure.setFlywheelVelocityManual(RPM.of(1500)),
                                intake.extendIntake(),
                                Commands.run(drive::stopWithX))
                        .withName("Shoot Manual Stop")
                        .andThen(superstructure.runFlywheelVelocityManual())
                        .andThen(intake.stopRollerCommand()));
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
                        drive, OIController.driveTranslationX(), OIController.driveTranslationY())));
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
        OIController.zeroDrivebase().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        //        OIController.manualHold().onTrue(Commands.runOnce(drive::stopWithX, drive));
        // OIController.start().onTrue(Commands.runOnce(resetGyro,
        // drive).ignoringDisable(true));

        OIController.intake()
                .and(shootingTrigger.negate())
                .whileTrue(intake.intakeRollerCommand().alongWith(indexer.index()))
                .onFalse(indexer.stop().alongWith(intake.idleRollerCommand()).unless(shootingTrigger::getAsBoolean));
        OIController.outtake()
                .and(shootingTrigger.negate())
                .whileTrue(intake.outtakeRollerCommand().alongWith(indexer.indexReverse()))
                .onFalse(indexer.stop().alongWith(intake.idleRollerCommand()).unless(shootingTrigger::getAsBoolean));
        OIController.xDrive().whileTrue(Commands.runOnce(drive::stopWithX, drive));

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

    public void resetSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
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
}
