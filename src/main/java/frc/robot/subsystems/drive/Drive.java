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
    static final double ODOMETRY_FREQUENCY = 125;

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

    private Pose2d[] smartAlignPath = new Pose2d[] {};
    private final Consumer<Pose2d> resetSimulationPoseCallBack;

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

        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        PhoenixOdometryThread.getInstance().start();

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

        // Log the full planned path so it draws a continuous line in AdvantageScope
        Logger.recordOutput("Drive/SmartAlignPath", smartAlignPath);
        Logger.recordOutput("Odometry/Robot3d", getPose3d(getPose()));

        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});

            // Clear trajectories when disabled
            smartAlignPath = new Pose2d[] {};
            Logger.recordOutput("Drive/ActiveTarget", new Pose2d());
        }

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    }

    public void runVelocity(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

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

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public Command align(APTarget target) {
        return this.run(() -> {
                    Logger.recordOutput("Drive/ActiveTarget", target.getReference()); // Log the immediate active target
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

    public Command swoop(Supplier<APTarget> targetSupplier) {
        return this.run(() -> {
                    APTarget target = targetSupplier.get();
                    Logger.recordOutput("Drive/ActiveTarget", target.getReference()); // Log the dynamic target

                    Pose2d currentPose = getPose();
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
                .until(() -> autopilot.atTarget(getPose(), targetSupplier.get()))
                .finallyDo(this::stop);
    }

    public Command alignWithSpin(APTarget target, Supplier<Boolean> allowSpin) {
        return this.run(() -> {
                    Logger.recordOutput("Drive/ActiveTarget", target.getReference());
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

    /**
     * * NEW Helper Method: Clamps a Pose2d so that the robot's mathematical center is not allowed to be placed closer
     * to the wall than its physical bumper radius.
     */
    public Pose2d clampPoseToField(Pose2d pose) {
        double margin = DRIVE_BASE_RADIUS + 0.15; // 15cm safety buffer from the wall

        double clampedX = Math.max(margin, Math.min(FieldConstants.fieldLength - margin, pose.getX()));
        double clampedY = Math.max(margin, Math.min(FieldConstants.fieldWidth - margin, pose.getY()));

        return new Pose2d(clampedX, clampedY, pose.getRotation());
    }

    public Command smartAlign(Pose2d rawTarget) {
        return Commands.defer(
                () -> {
                    // Pre-clamp the target so it's physically reachable, preventing the robot from crashing into walls
                    Pose2d target = clampPoseToField(rawTarget);

                    Pose2d current = getPose();
                    Translation2d currentTrans = current.getTranslation();
                    Translation2d targetTrans = target.getTranslation();

                    double hubX = FieldConstants.LinesVertical.hubCenter;
                    double oppHubX = FieldConstants.LinesVertical.oppHubCenter;
                    double centerDeltaY = FieldConstants.fieldWidth / 2.0;

                    double bumpDepth = FieldConstants.LeftBump.depth;
                    double obsHalfY = 3.5;

                    double obstacleX = -1;

                    if (pathIntersectsRect(
                            currentTrans,
                            targetTrans,
                            hubX - bumpDepth / 2.0,
                            hubX + bumpDepth / 2.0,
                            centerDeltaY - obsHalfY,
                            centerDeltaY + obsHalfY)) {
                        obstacleX = hubX;
                    } else if (pathIntersectsRect(
                            currentTrans,
                            targetTrans,
                            oppHubX - bumpDepth / 2.0,
                            oppHubX + bumpDepth / 2.0,
                            centerDeltaY - obsHalfY,
                            centerDeltaY + obsHalfY)) {
                        obstacleX = oppHubX;
                    }

                    if (obstacleX != -1) {
                        // Updated to exactly center the waypoints in the open zones based on 2026 CAD
                        double bottomTrenchY = 0.68;
                        double bottomBumpY = 2.58;
                        double topBumpY = FieldConstants.fieldWidth - 2.58;
                        double topTrenchY = FieldConstants.fieldWidth - 0.68;

                        // ALWAYS evaluate all 4 safe paths. No more forcing the trench.
                        double[][] candidatePaths = new double[][] {
                            {bottomTrenchY, 1.0},
                            {bottomBumpY, 0.0},
                            {topBumpY, 0.0},
                            {topTrenchY, 1.0}
                        };

                        double bestY = candidatePaths[0][0];
                        double minDistance = Double.MAX_VALUE;

                        for (double[] candidate : candidatePaths) {
                            double y = candidate[0];

                            // BUG FIX: True Euclidean distance to calculate the actual V-shaped path length
                            double dist = Math.hypot(obstacleX - current.getX(), y - current.getY())
                                    + Math.hypot(target.getX() - obstacleX, target.getY() - y);

                            if (dist < minDistance) {
                                minDistance = dist;
                                bestY = y;
                            }
                        }

                        double marginX = 1.0;
                        double beforeX = obstacleX - (current.getX() < obstacleX ? marginX : -marginX);
                        double afterX = obstacleX + (target.getX() > obstacleX ? marginX : -marginX);

                        double entryX = (beforeX + current.getX()) / 2.0;
                        double exitX = (afterX + target.getX()) / 2.0;

                        Rotation2d rot = target.getRotation();

                        Pose2d w1 = new Pose2d(entryX, bestY, rot);
                        Pose2d w2 = new Pose2d(obstacleX, bestY, rot);
                        Pose2d w3 = new Pose2d(exitX, bestY, rot);

                        // Publish to array so AdvantageScope renders it as a continuous line
                        smartAlignPath = new Pose2d[] {current, w1, w2, w3, target};

                        return align(new APTarget(w1))
                                .andThen(align(new APTarget(w2)))
                                .andThen(align(new APTarget(w3)))
                                .andThen(align(new APTarget(target)))
                                .finallyDo(() -> smartAlignPath = new Pose2d[] {}); // Clear line when finished
                    }

                    smartAlignPath = new Pose2d[] {current, target};
                    return align(new APTarget(target)).finallyDo(() -> smartAlignPath = new Pose2d[] {});
                },
                Set.of(this));
    }

    private boolean pathIntersectsRect(
            Translation2d start, Translation2d end, double minX, double maxX, double minY, double maxY) {
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(5.0).andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(5.0).andThen(sysId.dynamic(direction));
    }

    public Command sysIdQuasistaticTurning(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterizationTurning(0.0)).withTimeout(1.0).andThen(sysIdTurning.quasistatic(direction));
    }

    public Command sysIdDynamicTurning(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterizationTurning(0.0)).withTimeout(1.0).andThen(sysIdTurning.dynamic(direction));
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void setPose(Pose2d pose) {
        if (pose == null) {
            return;
        }
        resetSimulationPoseCallBack.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

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

        double marginX = DRIVE_BASE_RADIUS + 0.15;

        if (dx < (bumpDepth / 2.0 + marginX) && Math.abs(translation.getY() - centerDeltaY) < obsHalfY) {
            if (Math.abs(translation.getY() - centerDeltaY) > hubHalfY) {
                return true;
            }
        }

        return false;
    }

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

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }
}
