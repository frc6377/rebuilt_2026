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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;
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
    private static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.81);
    private static final int DEFAULT_TRAJECTORY_POINTS = 30;
    private static final Time MAX_TRAJECTORY_TIME = Seconds.of(3.0);

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
    private Time autoFireInterval = Seconds.of(1.0 / 3.0);

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

    public LinearVelocity calculateLaunchVelocity(AngularVelocity flywheelVelocity) {
        // v = (v_wheel + v_surface_stationary) / 2
        // For a single-wheel shooter, the ball rolls against a stationary hood,
        // so its center-of-mass velocity is half the wheel's tangential speed.
        return MetersPerSecond.of(
                (flywheelVelocity.in(RadiansPerSecond) * flywheelRadiusMeters.get() / 2.0) * launchEfficiency.get());
    }

    public LinearVelocity getCurrentLaunchVelocity(ShooterSide side) {
        return calculateLaunchVelocity(
                RPM.of(channel(side).flywheelVelocityRPMSupplier.get()));
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
        return launchFromSide(side, getCurrentLaunchVelocity(side), getHoodAngle());
    }

    /** Launches from the specified side with a custom launch velocity. */
    public GamePieceProjectile launchGamePiece(ShooterSide side, LinearVelocity launchVelocity) {
        return launchFromSide(side, launchVelocity, getHoodAngle());
    }

    /** Launches from the specified side with custom velocity and angle. */
    public GamePieceProjectile launchGamePiece(ShooterSide side, LinearVelocity launchVelocity, Angle launchAngle) {
        return launchFromSide(side, launchVelocity, launchAngle);
    }

    /** Launches one ball from each shooter. Requires at least 2 balls in hopper for both to fire. */
    public GamePieceProjectile[] launchBothGamePieces() {
        GamePieceProjectile leftProjectile = launchGamePiece(ShooterSide.LEFT);
        GamePieceProjectile rightProjectile = launchGamePiece(ShooterSide.RIGHT);
        return new GamePieceProjectile[] {leftProjectile, rightProjectile};
    }

    private GamePieceProjectile launchFromSide(ShooterSide side, LinearVelocity launchVelocity, Angle hoodAngle) {
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
                        launchVelocity,
                        hoodAngle)
                .enableBecomesGamePieceOnFieldAfterTouchGround();

        projectile.withProjectileTrajectoryDisplayCallBack(trajectory -> {
            ch.lastTrajectory = trajectory.toArray(new Pose3d[0]);
            Logger.recordOutput("Shooter/Sim/" + side.logName() + "/Trajectory", ch.lastTrajectory);
        });

        // Do not convert landed shots into dyn4j field bodies — that piles up physics objects
        // and dominates SimulatedArena.simulationPeriodic() cost during long sims / FullAuto.
        // Wait, I just added it back as requested by the user.
        // We should ensure it doesn't cause too much lag if many balls are fired.

        SimulatedArena.getInstance().addGamePieceProjectile(projectile);

        // Scoring logic: when the projectile hits the hub, spit it back out
        projectile
                .withTargetPosition(() -> FieldConstants.Hub.topCenterPoint)
                .withTargetTolerance(new Translation3d(FieldConstants.Hub.width, FieldConstants.Hub.width, 1.0))
                .withHitTargetCallBack(() -> {
                    // Start from the middle of the hub
                    Translation2d hubPos = FieldConstants.getHubPosition();
                    // neutral zone is towards the center of the field
                    Rotation2d spitDirection =
                            Rotation2d.fromDegrees(hubPos.getX() < FieldConstants.fieldLength / 2.0 ? 0 : 180);

                    GamePieceProjectile spitBall = new GamePieceProjectile(
                            gamePieceInfo,
                            hubPos,
                            new Translation2d(),
                            new ChassisSpeeds(),
                            spitDirection,
                            Meters.of(FieldConstants.Hub.innerHeight),
                            MetersPerSecond.of(4.0),
                            Degrees.of(15.0));
                    SimulatedArena.getInstance().addGamePieceProjectile(spitBall);
                });

        Logger.recordOutput("Shooter/Sim/" + side.logName() + "/LaunchVelocityMPS", launchVelocity.in(MetersPerSecond));
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
        Optional<Time> flightTime = calculateTimeOfFlight(state.initialHeight, state.verticalVelocity);

        if (flightTime.isEmpty()) {
            ch.lastTrajectory = new Pose3d[0];
            Logger.recordOutput("Shooter/Sim/" + side.logName() + "/PreviewTrajectory", ch.lastTrajectory);
            return ch.lastTrajectory;
        }

        Time totalTime = flightTime.get();
        if (totalTime.gt(MAX_TRAJECTORY_TIME)) {
            totalTime = MAX_TRAJECTORY_TIME;
        }
        ch.lastTrajectory = generateTrajectoryPoints(state, totalTime, numPoints);
        Logger.recordOutput("Shooter/Sim/" + side.logName() + "/PreviewTrajectory", ch.lastTrajectory);
        return ch.lastTrajectory;
    }

    private TrajectoryState calculateTrajectoryState(ShooterSide side) {
        Translation2d robotPosition = robotPositionSupplier.get();
        Rotation2d robotRotation = robotRotationSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

        LinearVelocity launchVelocity = getCurrentLaunchVelocity(side);
        Angle hoodAngle = getHoodAngle();
        Distance height = getShooterHeight();

        LinearVelocity launchHorizontalVelocity = launchVelocity.times(Math.cos(hoodAngle.in(Radians)));
        LinearVelocity launchVerticalVelocity = launchVelocity.times(Math.sin(hoodAngle.in(Radians)));

        Translation2d shooterOffset = getShooterOffset(side);
        Translation2d chassisVelocity =
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

        Translation2d shooterRotationalVelocity = shooterOffset
                .rotateBy(robotRotation)
                .rotateBy(Rotation2d.fromDegrees(90))
                .times(chassisSpeeds.omegaRadiansPerSecond);

        Translation2d totalHorizontalVelocity = chassisVelocity
                .plus(shooterRotationalVelocity)
                .plus(new Translation2d(launchHorizontalVelocity.in(MetersPerSecond), robotRotation));

        Translation2d launchPosition = robotPosition.plus(shooterOffset.rotateBy(robotRotation));

        return new TrajectoryState(
                Meters.of(launchPosition.getX()),
                Meters.of(launchPosition.getY()),
                height,
                MetersPerSecond.of(totalHorizontalVelocity.getX()),
                MetersPerSecond.of(totalHorizontalVelocity.getY()),
                launchVerticalVelocity);
    }

    private Optional<Time> calculateTimeOfFlight(Distance initialHeight, LinearVelocity verticalVelocity) {
        double g = GRAVITY.in(MetersPerSecondPerSecond);
        double v0 = verticalVelocity.in(MetersPerSecond);
        double y0 = initialHeight.in(Meters);

        double a = 0.5 * g;
        double b = -v0;
        double c = -y0;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return Optional.empty();
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);
        double t1 = (-b + sqrtDiscriminant) / (2 * a);
        double t2 = (-b - sqrtDiscriminant) / (2 * a);

        if (t1 > 0 && t2 > 0) {
            return Optional.of(Seconds.of(Math.min(t1, t2)));
        } else if (t1 > 0) {
            return Optional.of(Seconds.of(t1));
        } else if (t2 > 0) {
            return Optional.of(Seconds.of(t2));
        }

        return Optional.empty();
    }

    private Pose3d[] generateTrajectoryPoints(TrajectoryState state, Time totalTime, int numPoints) {
        Pose3d[] trajectory = new Pose3d[numPoints];
        Time dt = totalTime.div(numPoints - 1);

        for (int i = 0; i < numPoints; i++) {
            Time t = dt.times(i);
            Translation3d position = state.getPositionAtTime(t);
            double z = Math.max(0, position.getZ());
            trajectory[i] = new Pose3d(position.getX(), position.getY(), z, new Rotation3d());
        }

        return trajectory;
    }

    private static class TrajectoryState {
        public final Distance initialX, initialY, initialHeight;
        public final LinearVelocity horizontalVelocityX, horizontalVelocityY, verticalVelocity;

        public TrajectoryState(
                Distance initialX,
                Distance initialY,
                Distance initialHeight,
                LinearVelocity horizontalVelocityX,
                LinearVelocity horizontalVelocityY,
                LinearVelocity verticalVelocity) {
            this.initialX = initialX;
            this.initialY = initialY;
            this.initialHeight = initialHeight;
            this.horizontalVelocityX = horizontalVelocityX;
            this.horizontalVelocityY = horizontalVelocityY;
            this.verticalVelocity = verticalVelocity;
        }

        Translation3d getPositionAtTime(Time t) {
            double time = t.in(Seconds);
            double x = initialX.plus(Meters.of(horizontalVelocityX.in(MetersPerSecond) * time))
                    .in(Meters);
            double y = initialY.plus(Meters.of(horizontalVelocityY.in(MetersPerSecond) * time))
                    .in(Meters);
            double z = initialHeight
                    .plus(Meters.of(verticalVelocity.in(MetersPerSecond) * time))
                    .minus(Meters.of(0.5 * GRAVITY.in(MetersPerSecondPerSecond) * time * time))
                    .in(Meters);
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
        Optional<Time> flightTime = calculateTimeOfFlight(state.initialHeight, state.verticalVelocity);
        if (flightTime.isEmpty()) {
            return null;
        }

        return state.getPositionAtTime(flightTime.get());
    }

    public Translation3d getPositionAtTime(ShooterSide side, Time time) {
        if (time.lt(Seconds.of(0))) {
            return null;
        }

        TrajectoryState state = calculateTrajectoryState(side);
        Optional<Time> flightTime = calculateTimeOfFlight(state.initialHeight, state.verticalVelocity);
        if (flightTime.isEmpty() || time.gt(flightTime.get())) {
            return null;
        }

        return state.getPositionAtTime(time);
    }

    public Distance getHorizontalRange(ShooterSide side) {
        Translation3d landingPosition = getPredictedLandingPosition(side);
        if (landingPosition == null) {
            return Meters.of(-1);
        }

        TrajectoryState state = calculateTrajectoryState(side);
        double dx = landingPosition.getX() - state.initialX.in(Meters);
        double dy = landingPosition.getY() - state.initialY.in(Meters);
        return Meters.of(Math.sqrt(dx * dx + dy * dy));
    }

    public Distance getMaxHeight(ShooterSide side) {
        TrajectoryState state = calculateTrajectoryState(side);
        Time timeAtMaxHeight =
                Seconds.of(state.verticalVelocity.in(MetersPerSecond) / GRAVITY.in(MetersPerSecondPerSecond));

        if (timeAtMaxHeight.lte(Seconds.of(0))) {
            return state.initialHeight;
        }

        double t = timeAtMaxHeight.in(Seconds);
        return state.initialHeight
                .plus(Meters.of(state.verticalVelocity.in(MetersPerSecond) * t))
                .minus(Meters.of(0.5 * GRAVITY.in(MetersPerSecondPerSecond) * t * t));
    }

    public boolean willHitTarget(ShooterSide side, Translation3d targetCenter, Translation3d targetSize) {
        TrajectoryState state = calculateTrajectoryState(side);
        Optional<Time> flightTime = calculateTimeOfFlight(state.initialHeight, state.verticalVelocity);
        if (flightTime.isEmpty()) {
            return false;
        }

        Time totalTime = flightTime.get();
        int checkPoints = 20;

        for (int i = 0; i <= checkPoints; i++) {
            Time t = totalTime.times(i / (double) checkPoints);
            Translation3d pos = state.getPositionAtTime(t);

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
                getCurrentLaunchVelocity(side),
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

    public void setAutoFireInterval(Time interval) {
        this.autoFireInterval = interval;
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
            if (autoFireImmediate || autoFireTimer.hasElapsed(autoFireInterval.in(Seconds))) {
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

    public void enableAutoFire(BooleanSupplier indexerRunningSupplier, Time interval) {
        setIndexerRunningSupplier(indexerRunningSupplier);
        setAutoFireInterval(interval);
        setAutoFireEnabled(true);
    }

    public void enableAutoFireFromExternalAmmo(
            BooleanSupplier indexerRunningSupplier,
            BooleanSupplier ammoAvailable,
            BooleanSupplier ammoConsume,
            Time interval) {
        setExternalAmmoSource(ammoAvailable, ammoConsume);
        setIndexerRunningSupplier(indexerRunningSupplier);
        setAutoFireInterval(interval);
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
