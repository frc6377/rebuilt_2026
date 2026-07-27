// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PhoenixUtil;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {
    private Supplier<Pose2d> poseSupplier;

    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY = 125; // TODO: We should figure out a more permanent solution than this but
    // for now it's ok
    // new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ?
    // 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    private static final double BUMPER_LENGTH = 33.405;
    private static final double BUMPER_WIDTH = 37.75;

    // Autopilot constants
    private static final APConstraints kAutopilotConstraints =
            new APConstraints().withAcceleration(8.0).withJerk(4);

    private static final APProfile kAutopilotProfile = new APProfile(kAutopilotConstraints)
            .withErrorXY(Meters.of(0.03))
            .withErrorTheta(Degrees.of(1.0))
            .withBeelineRadius(Meters.of(0.15));

    private final Autopilot autopilot = new Autopilot(kAutopilotProfile);

    private static final RobotConfig PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            getModuleTranslations());

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
            .withBumperSize(Inches.of(BUMPER_LENGTH), Inches.of(BUMPER_WIDTH))
            .withCustomModuleTranslations(getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getKrakenX44(1),
                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                    PhoenixUtil.SIM_STEER_GEAR_RATIO,
                    Volts.of(PhoenixUtil.SIM_DRIVE_FRICTION_VOLTS),
                    Volts.of(PhoenixUtil.SIM_STEER_FRICTION_VOLTS),
                    Meters.of(TunerConstants.FrontLeft.WheelRadius),
                    KilogramSquareMeters.of(PhoenixUtil.SIM_STEER_INERTIA_KGM2),
                    WHEEL_COF));

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final SysIdRoutine sysIdTurning;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private Pose3d[] smartAlignWaypoints = new Pose3d[] {};
    private final Consumer<Pose2d> resetSimulationPoseCallBack; // TODO: Needs io interface sim should not be here

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;
        this.poseSupplier = this::getPose;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                () -> poseSupplier.get(),
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> SignalLogger.writeString("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));

        sysIdTurning = new SysIdRoutine(
                new SysIdRoutine.Config(null, null, null, (state) -> {
                    SignalLogger.writeString("SwerveTurn/state", state.toString());
                    Logger.recordOutput("Odometry/SysID Mode/Turn SysID", state.toString());
                }),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterizationTurning(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                "Drive/CurrentCommand",
                this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None");
        Logger.recordOutput("Drive/SmartAlignWaypoints", smartAlignWaypoints);
        Logger.recordOutput("Odometry/Robot3d", getPose3d(getPose()));

        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    public void runCharacterizationTurning(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterizationTurning(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public Command align(APTarget target) {
        return this.run(() -> {
                    Pose2d currentPose = getPose();
                    ChassisSpeeds robotRelativeSpeeds = getChassisSpeeds();

                    var result = autopilot.calculate(currentPose, robotRelativeSpeeds, target);

                    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            result.vx().in(MetersPerSecond),
                            result.vy().in(MetersPerSecond),
                            result.targetAngle()
                                            .minus(currentPose.getRotation())
                                            .getRadians()
                                    * 8.0,
                            currentPose.getRotation());

                    runVelocity(fieldRelativeSpeeds);
                })
                .until(() -> autopilot.atTarget(getPose(), target))
                .finallyDo(this::stop);
    }
    /**
     * * Dynamically aligns to a target that updates continuously. Perfect for sweeping through clusters of moving or
     * disappearing objects.
     */
    public Command swoop(Supplier<APTarget> targetSupplier) {
        return this.run(() -> {
                    Pose2d currentPose = getPose();
                    APTarget target = targetSupplier.get();

                    var result = autopilot.calculate(currentPose, getChassisSpeeds(), target);

                    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            result.vx().in(MetersPerSecond),
                            result.vy().in(MetersPerSecond),
                            result.targetAngle()
                                            .minus(currentPose.getRotation())
                                            .getRadians()
                                    * 8.0,
                            currentPose.getRotation());

                    runVelocity(fieldRelativeSpeeds);
                })
                // Stop when we reach the final target (or if the supplier returns our current pose)
                .until(() -> autopilot.atTarget(getPose(), targetSupplier.get()))
                .finallyDo(this::stop);
    }
    /** Aligns to target while spinning continuously, suppressing spin when entering trench or bump zones. */
    public Command alignWithSpin(APTarget target, Supplier<Boolean> allowSpin) {
        return this.run(() -> {
                    Pose2d currentPose = getPose();
                    ChassisSpeeds robotRelativeSpeeds = getChassisSpeeds();

                    var result = autopilot.calculate(currentPose, robotRelativeSpeeds, target);

                    double omega;
                    if (allowSpin.get() && !isInsideTrenchOrBump(currentPose.getTranslation())) {
                        omega = getMaxAngularSpeedRadPerSec() * 0.4;
                    } else {
                        omega = result.targetAngle()
                                        .minus(currentPose.getRotation())
                                        .getRadians()
                                * 8.0;
                    }

                    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            result.vx().in(MetersPerSecond),
                            result.vy().in(MetersPerSecond),
                            omega,
                            currentPose.getRotation());

                    runVelocity(fieldRelativeSpeeds);
                })
                .until(() -> autopilot.atTarget(getPose(), target))
                .finallyDo(this::stop);
    }

    /** Helper to check if robot translation is inside or approaching a trench or bump obstacle zone. */
    public boolean isInsideTrenchOrBump(Translation2d translation) {
        if (isNearBump(translation)) {
            return true;
        }
        double hubX = FieldConstants.LinesVertical.hubCenter;
        double oppHubX = FieldConstants.LinesVertical.oppHubCenter;
        double bumpDepth = FieldConstants.LeftBump.depth;

        double dxHub = Math.abs(translation.getX() - hubX);
        double dxOppHub = Math.abs(translation.getX() - oppHubX);
        double dx = Math.min(dxHub, dxOppHub);

        double marginX = DRIVE_BASE_RADIUS + 0.2;
        if (dx < (bumpDepth / 2.0 + marginX)) {
            double y = translation.getY();
            if (y < 1.2 || y > FieldConstants.fieldWidth - 1.2) {
                return true;
            }
        }
        return false;
    }

    public Command smartAlign(Pose2d target) {
        return Commands.defer(
                () -> {
                    Pose2d current = getPose();
                    Translation2d currentTrans = current.getTranslation();
                    Translation2d targetTrans = target.getTranslation();

                    double hubX = FieldConstants.LinesVertical.hubCenter;
                    double oppHubX = FieldConstants.LinesVertical.oppHubCenter;
                    double centerDeltaY = FieldConstants.fieldWidth / 2.0;

                    double hubHalfY = FieldConstants.Hub.width / 2.0;
                    double bumpDepth = FieldConstants.LeftBump.depth;
                    double obsHalfY = 3.5;

                    double obstacleX = -1;
                    boolean intersectsHub = false;

                    if (pathIntersectsRect(
                            currentTrans,
                            targetTrans,
                            hubX - bumpDepth / 2.0,
                            hubX + bumpDepth / 2.0,
                            centerDeltaY - obsHalfY,
                            centerDeltaY + obsHalfY)) {
                        obstacleX = hubX;
                        intersectsHub = pathIntersectsRect(
                                currentTrans,
                                targetTrans,
                                hubX - bumpDepth / 2.0,
                                hubX + bumpDepth / 2.0,
                                centerDeltaY - hubHalfY,
                                centerDeltaY + hubHalfY);
                    } else if (pathIntersectsRect(
                            currentTrans,
                            targetTrans,
                            oppHubX - bumpDepth / 2.0,
                            oppHubX + bumpDepth / 2.0,
                            centerDeltaY - obsHalfY,
                            centerDeltaY + obsHalfY)) {
                        obstacleX = oppHubX;
                        intersectsHub = pathIntersectsRect(
                                currentTrans,
                                targetTrans,
                                oppHubX - bumpDepth / 2.0,
                                oppHubX + bumpDepth / 2.0,
                                centerDeltaY - hubHalfY,
                                centerDeltaY + hubHalfY);
                    }

                    if (obstacleX != -1) {
                        double bottomTrenchY = 0.6;
                        double bottomBumpY = 2.5;
                        double topBumpY = FieldConstants.fieldWidth - 2.5;
                        double topTrenchY = FieldConstants.fieldWidth - 0.6;

                        double[][] candidatePaths;
                        if (intersectsHub) {
                            candidatePaths = new double[][] {
                                {bottomTrenchY, 1.0},
                                {topTrenchY, 1.0}
                            };
                        } else {
                            candidatePaths = new double[][] {
                                {bottomTrenchY, 1.0},
                                {bottomBumpY, 0.0},
                                {topBumpY, 0.0},
                                {topTrenchY, 1.0}
                            };
                        }

                        double bestY = candidatePaths[0][0];
                        boolean bestIsTrench = candidatePaths[0][1] == 1.0;
                        double minDistance = Double.MAX_VALUE;

                        for (double[] candidate : candidatePaths) {
                            double y = candidate[0];
                            double dist = Math.abs(current.getY() - y) + Math.abs(target.getY() - y);
                            if (dist < minDistance) {
                                minDistance = dist;
                                bestY = y;
                                bestIsTrench = candidate[1] == 1.0;
                            }
                        }

                        double marginX = 1.0;
                        double beforeX = obstacleX - (current.getX() < obstacleX ? marginX : -marginX);
                        double afterX = obstacleX + (target.getX() > obstacleX ? marginX : -marginX);

                        double entryX = (beforeX + current.getX()) / 2.0;
                        double exitX = (afterX + target.getX()) / 2.0;

                        double zHeight = bestIsTrench ? 0.0 : Units.inchesToMeters(5.02);

                        Pose3d w1 = new Pose3d(
                                entryX,
                                bestY,
                                0.0,
                                new Rotation3d(0, 0, target.getRotation().getRadians()));
                        Pose3d w2 = new Pose3d(
                                obstacleX,
                                bestY,
                                zHeight,
                                new Rotation3d(0, 0, target.getRotation().getRadians()));
                        Pose3d w3 = new Pose3d(
                                exitX,
                                bestY,
                                0.0,
                                new Rotation3d(0, 0, target.getRotation().getRadians()));

                        smartAlignWaypoints = new Pose3d[] {getPose3d(getPose()), w1, w2, w3, new Pose3d(target)};

                        return align(new APTarget(w1.toPose2d()))
                                .andThen(align(new APTarget(w2.toPose2d())))
                                .andThen(align(new APTarget(w3.toPose2d())))
                                .andThen(align(new APTarget(target)));
                    }

                    smartAlignWaypoints = new Pose3d[] {new Pose3d(target)};
                    return align(new APTarget(target));
                },
                Set.of(this));
    }
    /** Helper to check if a line segment intersects an axis-aligned bounding box. */
    private boolean pathIntersectsRect(
            Translation2d start, Translation2d end, double minX, double maxX, double minY, double maxY) {
        // Liang-Barsky or simple Cohen-Sutherland like check for segment-AABB intersection
        double x1 = start.getX();
        double y1 = start.getY();
        double x2 = end.getX();
        double y2 = end.getY();

        double tmin = 0.0;
        double tmax = 1.0;

        double dx = x2 - x1;
        if (dx == 0) {
            if (x1 < minX || x1 > maxX) return false;
        } else {
            double t1 = (minX - x1) / dx;
            double t2 = (maxX - x1) / dx;
            tmin = Math.max(tmin, Math.min(t1, t2));
            tmax = Math.min(tmax, Math.max(t1, t2));
        }

        double dy = y2 - y1;
        if (dy == 0) {
            if (y1 < minY || y1 > maxY) return false;
        } else {
            double t1 = (minY - y1) / dy;
            double t2 = (maxY - y1) / dy;
            tmin = Math.max(tmin, Math.min(t1, t2));
            tmax = Math.min(tmax, Math.max(t1, t2));
        }

        return tmax >= tmin;
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        };
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(states[i]);
        }
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(5.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(5.0).andThen(sysId.dynamic(direction));
    }

    public Command sysIdQuasistaticTurning(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterizationTurning(0.0)).withTimeout(1.0).andThen(sysIdTurning.quasistatic(direction));
    }

    public Command sysIdDynamicTurning(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterizationTurning(0.0)).withTimeout(1.0).andThen(sysIdTurning.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        if (pose == null) {
            return;
        }
        resetSimulationPoseCallBack.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Sets the pose supplier used by PathPlanner AutoBuilder. */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns the elevation of the field at the specified translation. */
    public double getElevation(Translation2d translation) {
        double hubX = FieldConstants.LinesVertical.hubCenter;
        double oppHubX = FieldConstants.LinesVertical.oppHubCenter;
        double centerDeltaY = FieldConstants.fieldWidth / 2.0;

        double hubHalfY = FieldConstants.Hub.width / 2.0;
        double bumpDepth = FieldConstants.LeftBump.depth;
        double bumpHeight = FieldConstants.LeftBump.height;
        double bumpTopWidth = FieldConstants.LeftBump.topWidth;
        double obsHalfY = hubHalfY + FieldConstants.LeftBump.width;

        double distanceToHubX = Math.abs(translation.getX() - hubX);
        double distanceToOppHubX = Math.abs(translation.getX() - oppHubX);
        double dx = Math.min(distanceToHubX, distanceToOppHubX);

        if (dx < bumpDepth / 2.0 && Math.abs(translation.getY() - centerDeltaY) < obsHalfY) {
            // It's in the Hub + Bump zone. If it's NOT in the Hub's Y-extent, it's on a bump.
            if (Math.abs(translation.getY() - centerDeltaY) > hubHalfY) {
                if (dx < bumpTopWidth / 2.0) {
                    return bumpHeight;
                } else {
                    double rampLength = (bumpDepth - bumpTopWidth) / 2.0;
                    return bumpHeight * (1.0 - (dx - bumpTopWidth / 2.0) / rampLength);
                }
            }
        }

        return 0.0;
    }

    /** Returns true if the robot pose is on or approaching a bump zone. */
    public boolean isNearBump(Translation2d translation) {
        double hubX = FieldConstants.LinesVertical.hubCenter;
        double oppHubX = FieldConstants.LinesVertical.oppHubCenter;
        double centerDeltaY = FieldConstants.fieldWidth / 2.0;

        double hubHalfY = FieldConstants.Hub.width / 2.0;
        double bumpDepth = FieldConstants.LeftBump.depth;
        double obsHalfY = hubHalfY + FieldConstants.LeftBump.width;

        double distanceToHubX = Math.abs(translation.getX() - hubX);
        double distanceToOppHubX = Math.abs(translation.getX() - oppHubX);
        double dx = Math.min(distanceToHubX, distanceToOppHubX);

        // Bumper offset margin so sensor toggles before front bumper collides with bump obstacle
        double marginX = DRIVE_BASE_RADIUS + 0.15;

        if (dx < (bumpDepth / 2.0 + marginX) && Math.abs(translation.getY() - centerDeltaY) < obsHalfY) {
            if (Math.abs(translation.getY() - centerDeltaY) > hubHalfY) {
                return true;
            }
        }

        return false;
    }

    /** Returns the 3D pose of the robot at the specified 2D pose, accounting for field elevation. */
    public Pose3d getPose3d(Pose2d pose) {
        Translation2d[] moduleTranslations = getModuleTranslations();
        double[] moduleElevations = new double[4];
        for (int i = 0; i < 4; i++) {
            Translation2d moduleFieldPos =
                    pose.getTranslation().plus(moduleTranslations[i].rotateBy(pose.getRotation()));
            moduleElevations[i] = getElevation(moduleFieldPos);
        }

        double z = (moduleElevations[0] + moduleElevations[1] + moduleElevations[2] + moduleElevations[3]) / 4.0;
        double wheelbase = moduleTranslations[0].getX() - moduleTranslations[2].getX();
        double trackwidth = moduleTranslations[0].getY() - moduleTranslations[1].getY();
        double pitch = -Math.atan2(
                (moduleElevations[0] + moduleElevations[1]) - (moduleElevations[2] + moduleElevations[3]),
                2.0 * wheelbase);
        double roll = Math.atan2(
                (moduleElevations[0] + moduleElevations[2]) - (moduleElevations[1] + moduleElevations[3]),
                2.0 * trackwidth);
        return new Pose3d(
                pose.getX(),
                pose.getY(),
                z,
                new Rotation3d(roll, pitch, pose.getRotation().getRadians()));
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }
}
