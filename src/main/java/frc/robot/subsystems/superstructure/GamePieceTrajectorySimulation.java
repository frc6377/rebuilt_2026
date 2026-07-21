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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class GamePieceTrajectorySimulation {
    public enum ShooterSide {
        LEFT("Left"),
        RIGHT("Right");

        private final String logName;

        ShooterSide(String logName) {
            this.logName = logName;
        }

        public String logName() {
            return logName;
        }
    }

    // Physics constants
    private static final double GRAVITY = 9.81; // Standard gravity (m/s²)
    private static final int DEFAULT_TRAJECTORY_POINTS = 30;
    private static final double MAX_TRAJECTORY_TIME = 3.0; // Maximum trajectory time (seconds)

    /**
     * Game piece info for the 2026 FUEL
     *
     * <p>FUEL specifications:
     *
     * <ul>
     *   <li>Diameter: 5.91 inches (15.0 cm)
     *   <li>Weight: 0.448-0.500 lb (~0.203-0.227 kg)
     *   <li>Material: High density foam ball
     * </ul>
     */
    public static final GamePieceOnFieldSimulation.GamePieceInfo FUEL_INFO =
            new GamePieceOnFieldSimulation.GamePieceInfo(
                    "Fuel", new Circle(0.075), Inches.of(5.91), Pounds.of(0.5), 2.0, 3.0, 0.6);

    private final GamePieceOnFieldSimulation.GamePieceInfo gamePieceInfo;

    private final Supplier<Translation2d> robotPositionSupplier;
    private final Supplier<Rotation2d> robotRotationSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private final ShooterChannel left;
    private final ShooterChannel right;

    private BooleanSupplier indexerRunningSupplier = () -> false;
    /** Optional ammo source (e.g. IntakeSimulation). When set, hopper is ignored for firing. */
    private BooleanSupplier externalAmmoAvailable = null;

    private BooleanSupplier externalAmmoConsume = null;
    private final Timer autoFireTimer = new Timer();
    /** ~3 balls/sec */
    private double autoFireIntervalSeconds = 1.0 / 3.0;

    private boolean autoFireEnabled = false;
    private boolean autoFireImmediate = false;
    private int gamePiecesLaunched = 0;
    private ShooterSide nextAutoFireSide = ShooterSide.LEFT;

    private final LoggedNetworkNumber ballsInHopper = new LoggedNetworkNumber("Shooter/Sim/BallsInHopper", 5);
    private boolean hopperEmptyStopsIndexer = true;

    private final LoggedNetworkNumber shooterHeightMeters;
    private final LoggedNetworkNumber flywheelRadiusMeters;
    private final LoggedNetworkNumber launchEfficiency;

    /**
     * Creates a new GamePieceTrajectorySimulation with independent left/right launch channels.
     *
     * @param driveSimulation The swerve drive simulation for robot state
     * @param leftFlywheelVelocityRPMSupplier Supplier for left flywheel velocity in RPM
     * @param rightFlywheelVelocityRPMSupplier Supplier for right flywheel velocity in RPM
     */
    public GamePieceTrajectorySimulation(
            SwerveDriveSimulation driveSimulation,
            Supplier<Double> leftFlywheelVelocityRPMSupplier,
            Supplier<Double> rightFlywheelVelocityRPMSupplier) {
        this(
                FUEL_INFO,
                () -> driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                () -> driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative,
                leftFlywheelVelocityRPMSupplier,
                rightFlywheelVelocityRPMSupplier);
    }

    /**
     * Creates a new GamePieceTrajectorySimulation with custom suppliers.
     *
     * @param gamePieceInfo Info about the game piece being launched
     * @param robotPositionSupplier Supplier for robot position on field
     * @param robotRotationSupplier Supplier for robot rotation
     * @param chassisSpeedsSupplier Supplier for chassis speeds (field-relative)
     * @param leftFlywheelVelocityRPMSupplier Supplier for left flywheel velocity in RPM
     * @param rightFlywheelVelocityRPMSupplier Supplier for right flywheel velocity in RPM
     */
    public GamePieceTrajectorySimulation(
            GamePieceOnFieldSimulation.GamePieceInfo gamePieceInfo,
            Supplier<Translation2d> robotPositionSupplier,
            Supplier<Rotation2d> robotRotationSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            Supplier<Double> leftFlywheelVelocityRPMSupplier,
            Supplier<Double> rightFlywheelVelocityRPMSupplier) {
        this.gamePieceInfo = gamePieceInfo;
        this.robotPositionSupplier = robotPositionSupplier;
        this.robotRotationSupplier = robotRotationSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;

        this.shooterHeightMeters =
                new LoggedNetworkNumber("Shooter/Sim/HeightMeters", ShooterConstants.shooterHeight.in(Meters));
        this.flywheelRadiusMeters =
                new LoggedNetworkNumber("Shooter/Sim/FlywheelRadiusMeters", ShooterConstants.flywheelRadius.in(Meters));
        this.launchEfficiency =
                new LoggedNetworkNumber("Shooter/Sim/LaunchEfficiency", ShooterConstants.launchEfficiency);

        this.left = new ShooterChannel(
                ShooterSide.LEFT,
                leftFlywheelVelocityRPMSupplier,
                ShooterConstants.shooterOffsetXLeft.in(Meters),
                ShooterConstants.shooterOffsetYLeft.in(Meters));
        this.right = new ShooterChannel(
                ShooterSide.RIGHT,
                rightFlywheelVelocityRPMSupplier,
                ShooterConstants.shooterOffsetXRight.in(Meters),
                ShooterConstants.shooterOffsetYRight.in(Meters));
    }

    private ShooterChannel channel(ShooterSide side) {
        return side == ShooterSide.LEFT ? left : right;
    }

    public double calculateLaunchVelocityMPS(double flywheelRPM) {
        double angularVelocityRadPerSec = flywheelRPM * 2.0 * Math.PI / 60.0;
        return angularVelocityRadPerSec * flywheelRadiusMeters.get() * launchEfficiency.get();
    }

    public double getCurrentLaunchVelocityMPS(ShooterSide side) {
        return calculateLaunchVelocityMPS(
                channel(side).flywheelVelocityRPMSupplier.get());
    }

    public Translation2d getShooterOffset(ShooterSide side) {
        return channel(side).getOffset();
    }

    public Distance getShooterHeight() {
        return Meters.of(shooterHeightMeters.get());
    }

    public Angle getHoodAngle() {
        return ShooterConstants.kFixedHoodAngle;
    }

    /** Launches from the next auto-fire side (alternating left/right). */
    public GamePieceProjectile launchGamePiece() {
        GamePieceProjectile projectile = launchGamePiece(nextAutoFireSide);
        nextAutoFireSide = nextAutoFireSide == ShooterSide.LEFT ? ShooterSide.RIGHT : ShooterSide.LEFT;
        return projectile;
    }

    /** Launches a game piece from the specified shooter side. */
    public GamePieceProjectile launchGamePiece(ShooterSide side) {
        return launchFromSide(side, getCurrentLaunchVelocityMPS(side), getHoodAngle());
    }

    /** Launches from the specified side with a custom launch velocity. */
    public GamePieceProjectile launchGamePiece(ShooterSide side, double launchVelocityMPS) {
        return launchFromSide(side, launchVelocityMPS, getHoodAngle());
    }

    /** Launches from the specified side with custom velocity and angle. */
    public GamePieceProjectile launchGamePiece(ShooterSide side, double launchVelocityMPS, double launchAngleDegrees) {
        return launchFromSide(side, launchVelocityMPS, Degrees.of(launchAngleDegrees));
    }

    /** Launches one ball from each shooter. Requires at least 2 balls in hopper for both to fire. */
    public GamePieceProjectile[] launchBothGamePieces() {
        GamePieceProjectile leftProjectile = launchGamePiece(ShooterSide.LEFT);
        GamePieceProjectile rightProjectile = launchGamePiece(ShooterSide.RIGHT);
        return new GamePieceProjectile[] {leftProjectile, rightProjectile};
    }

    private GamePieceProjectile launchFromSide(ShooterSide side, double launchVelocityMPS, Angle hoodAngle) {
        ShooterChannel ch = channel(side);
        Translation2d robotPosition = robotPositionSupplier.get();
        Rotation2d robotRotation = robotRotationSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
        Distance height = getShooterHeight();
        Translation2d offset = ch.getOffset();

        GamePieceProjectile projectile = new GamePieceProjectile(
                gamePieceInfo,
                robotPosition,
                offset,
                chassisSpeeds,
                robotRotation,
                height,
                MetersPerSecond.of(launchVelocityMPS),
                hoodAngle);

        projectile.withProjectileTrajectoryDisplayCallBack(trajectory -> {
            ch.lastTrajectory = trajectory.toArray(new Pose3d[0]);
            Logger.recordOutput("Shooter/Sim/" + side.logName() + "/Trajectory", ch.lastTrajectory);
        });
        // Do not convert landed shots into dyn4j field bodies — that piles up physics objects
        // and dominates SimulatedArena.simulationPeriodic() cost during long sims / FullAuto.

        SimulatedArena.getInstance().addGamePieceProjectile(projectile);

        Logger.recordOutput("Shooter/Sim/" + side.logName() + "/LaunchVelocityMPS", launchVelocityMPS);
        Logger.recordOutput("Shooter/Sim/" + side.logName() + "/LaunchAngleDegrees", hoodAngle.in(Degrees));
        Logger.recordOutput("Shooter/Sim/" + side.logName() + "/LaunchHeightMeters", height.in(Meters));
        Logger.recordOutput(
                "Shooter/Sim/" + side.logName() + "/LaunchPosition",
                new Pose2d(robotPosition.plus(offset.rotateBy(robotRotation)), robotRotation));

        return projectile;
    }

    /** Previews both left and right trajectories and returns the left preview. */
    public Pose3d[] previewTrajectory() {
        previewTrajectory(ShooterSide.RIGHT);
        return previewTrajectory(ShooterSide.LEFT);
    }

    public Pose3d[] previewTrajectory(ShooterSide side) {
        return previewTrajectory(side, DEFAULT_TRAJECTORY_POINTS);
    }

    public Pose3d[] previewTrajectory(ShooterSide side, int numPoints) {
        ShooterChannel ch = channel(side);
        TrajectoryState state = calculateTrajectoryState(side);
        Optional<Double> flightTime = calculateTimeOfFlight(state.initialHeight, state.verticalVelocity);

        if (flightTime.isEmpty()) {
            ch.lastTrajectory = new Pose3d[0];
            Logger.recordOutput("Shooter/Sim/" + side.logName() + "/PreviewTrajectory", ch.lastTrajectory);
            return ch.lastTrajectory;
        }

        double totalTime = Math.min(flightTime.get(), MAX_TRAJECTORY_TIME);
        ch.lastTrajectory = generateTrajectoryPoints(state, totalTime, numPoints);
        Logger.recordOutput("Shooter/Sim/" + side.logName() + "/PreviewTrajectory", ch.lastTrajectory);
        return ch.lastTrajectory;
    }

    private TrajectoryState calculateTrajectoryState(ShooterSide side) {
        Translation2d robotPosition = robotPositionSupplier.get();
        Rotation2d robotRotation = robotRotationSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

        double launchVelocityMPS = getCurrentLaunchVelocityMPS(side);
        double hoodAngleRad = getHoodAngle().in(Radians);
        double heightM = shooterHeightMeters.get();

        double launchHorizontalVelocity = launchVelocityMPS * Math.cos(hoodAngleRad);
        double launchVerticalVelocity = launchVelocityMPS * Math.sin(hoodAngleRad);

        Translation2d shooterOffset = getShooterOffset(side);
        Translation2d chassisVelocity =
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

        Translation2d shooterRotationalVelocity = shooterOffset
                .rotateBy(robotRotation)
                .rotateBy(Rotation2d.fromDegrees(90))
                .times(chassisSpeeds.omegaRadiansPerSecond);

        Translation2d totalHorizontalVelocity = chassisVelocity
                .plus(shooterRotationalVelocity)
                .plus(new Translation2d(launchHorizontalVelocity, robotRotation));

        Translation2d launchPosition = robotPosition.plus(shooterOffset.rotateBy(robotRotation));

        return new TrajectoryState(
                launchPosition.getX(),
                launchPosition.getY(),
                heightM,
                totalHorizontalVelocity.getX(),
                totalHorizontalVelocity.getY(),
                launchVerticalVelocity);
    }

    private Optional<Double> calculateTimeOfFlight(double initialHeight, double verticalVelocity) {
        double a = 0.5 * GRAVITY;
        double b = -verticalVelocity;
        double c = -initialHeight;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return Optional.empty();
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);
        double t1 = (-b + sqrtDiscriminant) / (2 * a);
        double t2 = (-b - sqrtDiscriminant) / (2 * a);

        if (t1 > 0 && t2 > 0) {
            return Optional.of(Math.min(t1, t2));
        } else if (t1 > 0) {
            return Optional.of(t1);
        } else if (t2 > 0) {
            return Optional.of(t2);
        }

        return Optional.empty();
    }

    private Pose3d[] generateTrajectoryPoints(TrajectoryState state, double totalTime, int numPoints) {
        Pose3d[] trajectory = new Pose3d[numPoints];
        double dt = totalTime / (numPoints - 1);

        for (int i = 0; i < numPoints; i++) {
            double t = i * dt;
            Translation3d position = state.getPositionAtTime(t, GRAVITY);
            double z = Math.max(0, position.getZ());
            trajectory[i] = new Pose3d(position.getX(), position.getY(), z, new Rotation3d());
        }

        return trajectory;
    }

    private record TrajectoryState(
            double initialX,
            double initialY,
            double initialHeight,
            double horizontalVelocityX,
            double horizontalVelocityY,
            double verticalVelocity) {

        Translation3d getPositionAtTime(double t, double gravity) {
            double x = initialX + horizontalVelocityX * t;
            double y = initialY + horizontalVelocityY * t;
            double z = initialHeight + verticalVelocity * t - 0.5 * gravity * t * t;
            return new Translation3d(x, y, z);
        }
    }

    public Pose3d[] getLastTrajectory(ShooterSide side) {
        return channel(side).lastTrajectory;
    }

    /** Returns the last trajectory from the most recently used auto-fire side's counterpart. Defaults to left. */
    public Pose3d[] getLastTrajectory() {
        return left.lastTrajectory.length > 0 ? left.lastTrajectory : right.lastTrajectory;
    }

    public Translation3d getPredictedLandingPosition(ShooterSide side) {
        TrajectoryState state = calculateTrajectoryState(side);
        Optional<Double> flightTime = calculateTimeOfFlight(state.initialHeight, state.verticalVelocity);
        if (flightTime.isEmpty()) {
            return null;
        }

        double t = flightTime.get();
        return new Translation3d(
                state.initialX() + state.horizontalVelocityX() * t,
                state.initialY() + state.horizontalVelocityY() * t,
                0);
    }

    public Translation3d getPositionAtTime(ShooterSide side, double timeSeconds) {
        if (timeSeconds < 0) {
            return null;
        }

        TrajectoryState state = calculateTrajectoryState(side);
        Optional<Double> flightTime = calculateTimeOfFlight(state.initialHeight, state.verticalVelocity);
        if (flightTime.isEmpty() || timeSeconds > flightTime.get()) {
            return null;
        }

        return state.getPositionAtTime(timeSeconds, GRAVITY);
    }

    public double getHorizontalRange(ShooterSide side) {
        Translation3d landingPosition = getPredictedLandingPosition(side);
        if (landingPosition == null) {
            return -1;
        }

        TrajectoryState state = calculateTrajectoryState(side);
        double dx = landingPosition.getX() - state.initialX();
        double dy = landingPosition.getY() - state.initialY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getMaxHeight(ShooterSide side) {
        TrajectoryState state = calculateTrajectoryState(side);
        double timeAtMaxHeight = state.verticalVelocity() / GRAVITY;

        if (timeAtMaxHeight <= 0) {
            return state.initialHeight();
        }

        return state.initialHeight()
                + state.verticalVelocity() * timeAtMaxHeight
                - 0.5 * GRAVITY * timeAtMaxHeight * timeAtMaxHeight;
    }

    public boolean willHitTarget(ShooterSide side, Translation3d targetCenter, Translation3d targetSize) {
        TrajectoryState state = calculateTrajectoryState(side);
        Optional<Double> flightTime = calculateTimeOfFlight(state.initialHeight, state.verticalVelocity);
        if (flightTime.isEmpty()) {
            return false;
        }

        double totalTime = flightTime.get();
        int checkPoints = 20;

        for (int i = 0; i <= checkPoints; i++) {
            double t = (i / (double) checkPoints) * totalTime;
            Translation3d pos = state.getPositionAtTime(t, GRAVITY);

            if (Math.abs(pos.getX() - targetCenter.getX()) <= targetSize.getX()
                    && Math.abs(pos.getY() - targetCenter.getY()) <= targetSize.getY()
                    && Math.abs(pos.getZ() - targetCenter.getZ()) <= targetSize.getZ()) {
                return true;
            }
        }

        return false;
    }

    public GamePieceProjectile createTargetedProjectile(
            ShooterSide side, Supplier<Translation3d> targetPosition, Translation3d tolerance, Runnable onHit) {
        ShooterChannel ch = channel(side);
        Translation2d robotPosition = robotPositionSupplier.get();
        Rotation2d robotRotation = robotRotationSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

        GamePieceProjectile projectile = new GamePieceProjectile(
                gamePieceInfo,
                robotPosition,
                ch.getOffset(),
                chassisSpeeds,
                robotRotation,
                getShooterHeight(),
                MetersPerSecond.of(getCurrentLaunchVelocityMPS(side)),
                getHoodAngle());

        projectile
                .withTargetPosition(targetPosition)
                .withTargetTolerance(tolerance)
                .withHitTargetCallBack(onHit)
                .withProjectileTrajectoryDisplayCallBack(
                        trajectoryHit -> {
                            ch.lastTrajectory = trajectoryHit.toArray(new Pose3d[0]);
                            Logger.recordOutput("Shooter/Sim/" + side.logName() + "/Trajectory", ch.lastTrajectory);
                        },
                        trajectoryMiss -> {
                            ch.lastTrajectory = trajectoryMiss.toArray(new Pose3d[0]);
                            Logger.recordOutput("Shooter/Sim/" + side.logName() + "/TrajectoryMiss", ch.lastTrajectory);
                        });

        return projectile;
    }

    // ==================== Auto-Fire Simulation ====================

    public void setIndexerRunningSupplier(BooleanSupplier indexerRunningSupplier) {
        this.indexerRunningSupplier = indexerRunningSupplier;
    }

    /**
     * Use an external ammo source (e.g. MapleSim IntakeSimulation) instead of the hopper. {@code consume} should return
     * true if a piece was successfully taken for this shot.
     */
    public void setExternalAmmoSource(BooleanSupplier available, BooleanSupplier consume) {
        this.externalAmmoAvailable = available;
        this.externalAmmoConsume = consume;
    }

    public void clearExternalAmmoSource() {
        this.externalAmmoAvailable = null;
        this.externalAmmoConsume = null;
    }

    private boolean usingExternalAmmo() {
        return externalAmmoAvailable != null && externalAmmoConsume != null;
    }

    private boolean hasAmmoToFire() {
        if (usingExternalAmmo()) {
            return externalAmmoAvailable.getAsBoolean();
        }
        return !hopperEmptyStopsIndexer || hasBalls();
    }

    private boolean consumeAmmoForShot() {
        if (usingExternalAmmo()) {
            return externalAmmoConsume.getAsBoolean();
        }
        int current = getBallsInHopper();
        if (current <= 0) {
            return false;
        }
        setBallsInHopper(current - 1);
        return true;
    }

    public void setAutoFireInterval(double intervalSeconds) {
        this.autoFireIntervalSeconds = intervalSeconds;
    }

    public void setAutoFireEnabled(boolean enabled) {
        this.autoFireEnabled = enabled;
        if (enabled) {
            autoFireTimer.restart();
            autoFireImmediate = true; // Fire first ball without waiting a full interval
        } else {
            autoFireTimer.stop();
            autoFireImmediate = false;
            clearExternalAmmoSource();
        }
    }

    public boolean isAutoFireEnabled() {
        return autoFireEnabled;
    }

    public int getGamePiecesLaunched() {
        return gamePiecesLaunched;
    }

    public void resetGamePiecesLaunched() {
        gamePiecesLaunched = 0;
    }

    public int getBallsInHopper() {
        return (int) ballsInHopper.get();
    }

    public void setBallsInHopper(int count) {
        ballsInHopper.set(Math.max(0, count));
    }

    public void addBalls(int count) {
        setBallsInHopper(getBallsInHopper() + count);
    }

    public boolean hasBalls() {
        return getBallsInHopper() > 0;
    }

    public boolean isEmpty() {
        return getBallsInHopper() <= 0;
    }

    public void setHopperEmptyStopsIndexer(boolean enabled) {
        this.hopperEmptyStopsIndexer = enabled;
    }

    public boolean shouldIndexerRun() {
        if (!hopperEmptyStopsIndexer) {
            return true;
        }
        return hasBalls();
    }

    /**
     * Updates the auto-fire simulation. Alternates left/right launchers each shot so both trajectories are exercised.
     * Preview arcs are only refreshed while a flywheel is spinning to avoid NT spam every cycle.
     */
    public void updateAutoFire() {
        int currentBalls = getBallsInHopper();

        Logger.recordOutput("Shooter/Sim/AutoFireEnabled", autoFireEnabled);
        Logger.recordOutput("Shooter/Sim/IndexerRunning", indexerRunningSupplier.getAsBoolean());
        Logger.recordOutput("Shooter/Sim/GamePiecesLaunched", gamePiecesLaunched);
        Logger.recordOutput("Shooter/Sim/BallsRemaining", currentBalls);
        Logger.recordOutput("Shooter/Sim/HopperEmpty", currentBalls <= 0);
        Logger.recordOutput("Shooter/Sim/UsingExternalAmmo", usingExternalAmmo());
        Logger.recordOutput("Shooter/Sim/HasAmmoToFire", hasAmmoToFire());
        Logger.recordOutput("Shooter/Sim/IndexerAllowed", shouldIndexerRun());
        Logger.recordOutput("Shooter/Sim/NextAutoFireSide", nextAutoFireSide.logName());

        boolean leftSpinning = left.flywheelVelocityRPMSupplier.get() > 100.0;
        boolean rightSpinning = right.flywheelVelocityRPMSupplier.get() > 100.0;
        if (leftSpinning) {
            previewTrajectory(ShooterSide.LEFT);
        }
        if (rightSpinning) {
            previewTrajectory(ShooterSide.RIGHT);
        }

        if (!autoFireEnabled) {
            return;
        }

        if (!hasAmmoToFire()) {
            autoFireTimer.restart();
            return;
        }

        boolean indexerRunning = indexerRunningSupplier.getAsBoolean();

        if (indexerRunning) {
            if (autoFireImmediate || autoFireTimer.hasElapsed(autoFireIntervalSeconds)) {
                if (!consumeAmmoForShot()) {
                    autoFireTimer.restart();
                    return;
                }
                launchGamePiece();
                gamePiecesLaunched++;
                autoFireImmediate = false;
                autoFireTimer.restart();
                Logger.recordOutput("Shooter/Sim/LastLaunchTime", Timer.getFPGATimestamp());
            }
        } else {
            autoFireTimer.restart();
        }
    }

    public void enableAutoFire(BooleanSupplier indexerRunningSupplier) {
        setIndexerRunningSupplier(indexerRunningSupplier);
        setAutoFireEnabled(true);
    }

    public void enableAutoFire(BooleanSupplier indexerRunningSupplier, double intervalSeconds) {
        setIndexerRunningSupplier(indexerRunningSupplier);
        setAutoFireInterval(intervalSeconds);
        setAutoFireEnabled(true);
    }

    public void enableAutoFireFromExternalAmmo(
            BooleanSupplier indexerRunningSupplier,
            BooleanSupplier ammoAvailable,
            BooleanSupplier ammoConsume,
            double intervalSeconds) {
        setExternalAmmoSource(ammoAvailable, ammoConsume);
        setIndexerRunningSupplier(indexerRunningSupplier);
        setAutoFireInterval(intervalSeconds);
        setAutoFireEnabled(true);
    }

    /** Per-shooter launch channel: independent offset, RPM supplier, and trajectory history. */
    private static final class ShooterChannel {
        private final Supplier<Double> flywheelVelocityRPMSupplier;
        private final LoggedNetworkNumber offsetXMeters;
        private final LoggedNetworkNumber offsetYMeters;
        private Pose3d[] lastTrajectory = new Pose3d[0];

        private ShooterChannel(
                ShooterSide side, Supplier<Double> flywheelVelocityRPMSupplier, double defaultX, double defaultY) {
            this.flywheelVelocityRPMSupplier = flywheelVelocityRPMSupplier;
            this.offsetXMeters = new LoggedNetworkNumber("Shooter/Sim/" + side.logName() + "/OffsetXMeters", defaultX);
            this.offsetYMeters = new LoggedNetworkNumber("Shooter/Sim/" + side.logName() + "/OffsetYMeters", defaultY);
        }

        private Translation2d getOffset() {
            return new Translation2d(offsetXMeters.get(), offsetYMeters.get());
        }
    }
}
