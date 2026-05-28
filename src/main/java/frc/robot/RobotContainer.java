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

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomy.AutonomyManager;
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
import java.util.Objects;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Autonomous-first robot container. No joystick/OI bindings are installed in production runtime. */
public class RobotContainer {
    protected final Superstructure superstructure;
    protected final Intake intake;

    private final Drive drive;
    private final Vision vision;
    private final Indexer indexer;
    private final SwerveDriveSimulation driveSimulation;
    private final LoggedDashboardChooser<Command> autoChooser;
    private final AutonomyManager autonomyManager;

    public RobotContainer() {
        RobotState.create();

        switch (Constants.currentMode) {
            case REAL:
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

        superstructure = new Superstructure(intake::isRollerRunning, vision);

        if (Constants.currentMode == Constants.Mode.SIM) {
            superstructure.configureGamePieceSimulation(driveSimulation);
        }

        RobotState.getInstance().setPoseSupplier(drive::getPose);
        autonomyManager = new AutonomyManager(drive, vision, superstructure, intake, indexer, RobotState.getInstance());

        registerAutonomousSkillCommands();

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        configureAutonomousChooser();
    }

    private void registerAutonomousSkillCommands() {
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
        NamedCommands.registerCommand("Wait 5 seconds", Commands.waitSeconds(5));
        NamedCommands.registerCommand("Extend Intake", intake.extendIntake());
        NamedCommands.registerCommand("Intake", Commands.parallel(intake.intakeCommand(), indexer.index()));
        NamedCommands.registerCommand(
                "Index",
                Commands.runOnce(() -> indexer.setRunning(true)).withTimeout(3).andThen(indexer.stop()));
        NamedCommands.registerCommand("Auto Aim", DriveCommands.holdAngle(drive, superstructure::getTargetHeading));
        NamedCommands.registerCommand("Stop intake", intake.stopRollerCommand());
    }

    private void configureAutonomousChooser() {
        autoChooser.addOption("Autonomous Brain - One Cycle", createAutonomousBrainCommand(true));
        autoChooser.addOption("Autonomous Brain - Continuous", createAutonomousBrainCommand(false));

        if (Constants.currentMode == Constants.Mode.SIM) {
            autoChooser.addOption("Shooter Tuning Sim", new ShooterCalibrationCommand(superstructure, driveSimulation));
        }

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
    }

    private Command createAutonomousBrainCommand(boolean finishAfterOneScoredCycle) {
        return autonomyManager.createBrainCommand(finishAfterOneScoredCycle);
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public Command getContinuousAutonomousCommand() {
        return createAutonomousBrainCommand(false);
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
