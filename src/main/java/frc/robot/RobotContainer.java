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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCalibrationCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimberIO;
import frc.robot.subsystems.climb.ClimberIOReal;
import frc.robot.subsystems.climb.ClimberIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.extender.ExtenderIO;
import frc.robot.subsystems.intake.extender.ExtenderIOReal;
import frc.robot.subsystems.intake.extender.ExtenderIOSim;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOReal;
import frc.robot.subsystems.intake.roller.RollerIOSim;
import frc.robot.subsystems.superstructure.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.*;
import frc.robot.util.OILayer.OI;
import frc.robot.util.OILayer.OIKeyboard;
import frc.robot.util.OILayer.OIXbox;
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
    private final Superstructure superstructure;

    private final Drive drive;

    // Jay was here and basiclly is the reason that this code works <3

    private final Vision vision;
    private final Climb climb;

    private final OI oi;
    private final Intake intake;
    private final OI OIController;
    private final Indexer indexer;
    private final SwerveDriveSimulation
            driveSimulation; // Only used in simulation, but declared here for easy access by subsystems that need it
    private final RobotState robotState;

    private final boolean usingController;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        robotState = RobotState.create();

        usingController = true;

        if (usingController && Constants.currentMode != Constants.Mode.SIM) {
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
                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
                climb = new Climb(new ClimberIOReal());

                intake = new Intake(
                        Constants.EnabledSubsystems.kRoller ? new RollerIOReal() : new RollerIO() {},
                        Constants.EnabledSubsystems.kExtender ? new ExtenderIOReal() : new ExtenderIO() {});
                indexer = new Indexer(Constants.EnabledSubsystems.kIndexer ? new IndexerIOReal() : new IndexerIO() {});
                driveSimulation = null;
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                intake = new Intake(
                        Constants.EnabledSubsystems.kRoller ? new RollerIOSim(driveSimulation) : new RollerIO() {},
                        Constants.EnabledSubsystems.kExtender ? new ExtenderIOSim() : new ExtenderIO() {});
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
                        driveSimulation::setSimulationWorldPose);
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                climb = new Climb(new ClimberIOSim());
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
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                climb = new Climb(new ClimberIO() {});
                intake = new Intake(new RollerIO() {}, new ExtenderIO() {});
                indexer = new Indexer(new IndexerIO() {});
                break;
        }
        oi = new OIXbox();

        superstructure = new Superstructure(intake::isRollerRunning);

        if (Constants.currentMode == Constants.Mode.SIM) {
            superstructure.configureGamePieceSimulation(driveSimulation);
        }
        NamedCommands.registerCommand("Spin Up Shooter", superstructure.setFlywheelVelocityCommand(RPM.of(3600)));
        NamedCommands.registerCommand(
                "Spin Up Shooter and Wait", superstructure.setFlywheelVelocityAndWaitCommand(RPM.of(3600)));
        NamedCommands.registerCommand(
                "Shoot", Commands.deadline(Commands.waitSeconds(5), superstructure.fireCommand()));
        // NamedCommands.registerCommand("Intake", Commands.deadline(intake.intakeCommand(), Commands.waitSeconds(6)));
        NamedCommands.registerCommand("Extend Intake", Commands.runOnce(intake::extendIntake));
        NamedCommands.registerCommand(
                "Intake",
                Commands.deadline(
                                Commands.waitSeconds(6),
                                Commands.parallel(
                                        intake.intakeCommand(), Commands.runOnce(() -> indexer.setRunning(true))))
                        .andThen(indexer.stop()));
        NamedCommands.registerCommand(
                "Index",
                Commands.runOnce(() -> indexer.setRunning(true)).withTimeout(3).andThen(indexer.stop()));
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
                () -> OIController.driveTranslationY().getAsDouble(),
                () -> OIController.driveTranslationX().getAsDouble(),
                () -> OIController.driveRotation().getAsDouble()));
        indexer.setDefaultCommand(indexer.index(() ->
                (intake.isRollerRunning() || superstructure.getLeftShooter().isRunning())));
        // // Lock to 0° when button is held
        // OIController.driveLock0()
        //         .whileTrue(DriveCommands.joystickDriveAtAngle(
        //                 drive,
        //                 () -> -OIController.driveTranslationY().getAsDouble(),
        //                 () -> -OIController.driveTranslationX().getAsDouble(),
        //                 () -> new Rotation2d()));

        OIController.spinUpShooter().whileTrue(superstructure.runFlywheelVelocityManual());

        // Manual fire (feeds piece when shooter is ready)
        OIController.fireShooter().whileTrue(superstructure.fireCommand()).onFalse(superstructure.stopUpgoerCommand());

        OIController.unjamShooter()
                .whileTrue(superstructure.unjamCommand())
                .onFalse(superstructure.stopUpgoerCommand());

        // Stop all components
        OIController.stopSuperstructure()
                .onTrue(superstructure.stopShooterCommand().alongWith(superstructure.stopUpgoerCommand()));

        OIController.shootSpeedLow().onTrue(superstructure.setFlywheelVelocityManual(RPM.of(2900)));
        OIController.shootSpeedMidLow().onTrue(superstructure.setFlywheelVelocityManual(RPM.of(3300)));
        OIController.shootSpeedMidHigh().onTrue(superstructure.setFlywheelVelocityManual(RPM.of(3600)));
        OIController.shootSpeedHigh().onTrue(superstructure.setFlywheelVelocityManual(RPM.of(3900)));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        OIController.zeroDrivebase().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        // OIController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // Example Coral Placement Code
        // TODO: delete these code for your own project
        if (Constants.currentMode == Constants.Mode.SIM) {
            // L4 placement
        //     controller.y().onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
        //             .addGamePieceProjectile(new ReefscapeCoralOnFly(
        //                     driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        //                     new Translation2d(0.4, 0),
        //                     driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        //                     driveSimulation.getSimulatedDriveTrainPose().getRotation(),
        //                     Meters.of(2),
        //                     MetersPerSecond.of(1.5),
        //                     Degrees.of(-80)))));
        //     // L3 placement
        //     controller.b().onTrue(Commands.runOnce(() -> SimulatedArena.getInstance()
        //             .addGamePieceProjectile(new ReefscapeCoralOnFly(
        //                     driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
        //                     new Translation2d(0.4, 0),
        //                     driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        //                     driveSimulation.getSimulatedDriveTrainPose().getRotation(),
        //                     Meters.of(1.35),
        //                     MetersPerSecond.of(1.5),
        //                     Degrees.of(-60)))));
            // Climb controls
        //     oi.climbExtend().whileTrue(climb.climbUp());
            oi.declimb().whileTrue(climb.climbUp());
            oi.climb_l1().onTrue(climb.setHeight(Inches.of(27))); // Should be 27
        //     oi.climbToFloor().onTrue(climb.setHeight(Inches.of(0)));

            //     oiKeyboard.climbExtend().whileTrue(climb.climbUp());
            //     oiKeyboard.declimb().whileTrue(climb.climbDown()).onTrue(Commands.print("V pressed - climbing
            // down"));
            //     oiKeyboard
            //             .climbToL1()
            //             .onTrue(Commands.print("M pressed - going to 27 inches"))
            //             .onTrue(climb.setHeight(Inches.of(27)));
            //     oiKeyboard
            //             .climbToFloor()
            //             .onTrue(Commands.print("Slash pressed - going to 0 inches"))
            //             .onTrue(climb.setHeight(Inches.of(0)));
        }
        OIController.intake().whileTrue(intake.intakeCommand());
        OIController.outtake().whileTrue(intake.outtakeRollerCommand());
        OIController.zeroIntake().onTrue(intake.zeroIntake());
        OIController.toggleIntakeState().onTrue(intake.toggleIntake());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
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
