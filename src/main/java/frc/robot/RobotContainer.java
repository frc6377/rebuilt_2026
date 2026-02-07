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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.state.RobotState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.upgoer.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.OILayer.OI;
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
    private final Drive drive;

    @SuppressWarnings("unused")
    private final Vision vision;

    private final Superstructure superstructure;
    private final Upgoer upgoer;
    private final RobotState robotState;

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);
    private final OI oi = new OIXbox();
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        robotState = RobotState.create();
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {});
                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
                superstructure = new Superstructure();
                upgoer = new Upgoer(new UpgoerIOKrakenX60());

                break;
            case SIM:

                // Sim robot, instantiate physics sim IO implementations
                driveSimulation =
                        new SwerveDriveSimulation(Drive.createMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
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
                superstructure = new Superstructure();
                upgoer = new Upgoer(new UpgoerIOSim());
                superstructure.configureGamePieceSimulation(driveSimulation);

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                superstructure = new Superstructure();
                upgoer = new Upgoer(new UpgoerIO() {});

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Add shooter calibration mode (simulation only)
        if (Constants.currentMode == Constants.Mode.SIM) {
            Command calibrationCommand =
                    superstructure.createShooterCalibrationCommand(driveSimulation, drive::setPose);
            if (calibrationCommand != null) {
                autoChooser.addOption("Shooter Calibration", calibrationCommand);
            }
        }

        // Configure the button bindings
        configureButtonBindings();

        robotState.setPoseSupplier(drive::getPose);
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
                () -> -oi.driveTranslationY().getAsDouble(),
                () -> -oi.driveTranslationX().getAsDouble(),
                () -> -oi.driveRotation().getAsDouble()));

        // Lock to 0° when A button is held
        controller
                .a()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> new Rotation2d()));

        // Spin up shooter using OI mapping (also previews trajectory in sim)
        Command spinUpCommand = superstructure.spinUpShooterCommand(
                drive, () -> -oi.driveTranslationY().getAsDouble(), () -> -oi.driveTranslationX()
                        .getAsDouble());
        if (Constants.currentMode == Constants.Mode.SIM && superstructure.hasGamePieceTrajectorySimulation()) {
            spinUpCommand = spinUpCommand.alongWith(Commands.run(() ->
                            superstructure.getGamePieceTrajectorySimulation().previewTrajectory())
                    .withName("PreviewTrajectory"));
        }
        oi.spinUpShooter().whileTrue(spinUpCommand).onFalse(superstructure.stopShooterCommand());

        // Fire shooter using OI mapping (only feeds when at speed and in valid zone, launches projectiles in sim)
        Command fireCommand = Commands.run(
                        () -> {
                            if (superstructure.atTargetVelocity() && superstructure.isInShootingZone(drive.getPose())) {
                                upgoer.setVelocity(UpgoerConstants.defaultFeedVelocity);
                            } else {
                                upgoer.stop();
                            }
                        },
                        upgoer)
                .withName("FireShooter");
        if (Constants.currentMode == Constants.Mode.SIM && superstructure.hasGamePieceTrajectorySimulation()) {
            fireCommand = fireCommand.alongWith(superstructure.simAutoFireHoldCommand(
                    () -> superstructure.atTargetVelocity() && superstructure.isInShootingZone(drive.getPose())));
        }
        oi.fireShooter().whileTrue(fireCommand).onFalse(upgoer.stopCommand());

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(
                        driveSimulation.getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
                // simulation
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
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
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
}
