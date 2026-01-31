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

package frc.robot.subsystems.shooter;

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
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Simulates game piece trajectory for the shooter using Maple-Sim's physics engine. Handles launching game pieces with
 * realistic projectile motion based on flywheel velocity and hood angle.
 *
 * <p>The simulation accounts for:
 *
 * <ul>
 *   <li>Flywheel velocity to linear launch speed conversion
 *   <li>Hood angle for launch trajectory
 *   <li>Robot chassis velocity contribution to projectile velocity
 *   <li>Projectile physics (gravity, no air drag)
 * </ul>
 */
public class GamePieceTrajectorySimulation {
    /**
     * Game piece info for the 2017 FUEL (am-5801).
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
                    "Fuel", // Game piece type name
                    new Circle(0.075), // Radius: 5.91"/2 = 2.955" = 0.075m
                    Inches.of(5.91), // Height (diameter for a sphere)
                    Kilograms.of(0.215), // Average mass: (0.203 + 0.227) / 2
                    2.0, // Linear damping (foam has some air resistance)
                    3.0, // Angular damping
                    0.6); // Coefficient of restitution (foam bounces moderately)

    // Game piece configuration
    private final GamePieceOnFieldSimulation.GamePieceInfo gamePieceInfo;

    // Robot state suppliers
    private final Supplier<Translation2d> robotPositionSupplier;
    private final Supplier<Rotation2d> robotRotationSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    // Shooter state suppliers
    private final Supplier<Double> flywheelVelocityRPMSupplier;
    private final Supplier<Double> hoodAngleDegreesSupplier;

    // Indexer state for automatic firing
    private BooleanSupplier indexerRunningSupplier = () -> false;
    private final Timer autoFireTimer = new Timer();
    private double autoFireIntervalSeconds = 1.0;
    private boolean autoFireEnabled = false;
    private int gamePiecesLaunched = 0;

    // Ball count / hopper simulation
    private final LoggedNetworkNumber ballsInHopper =
            new LoggedNetworkNumber("Shooter/Sim/BallsInHopper", 5); // Default to 5 balls
    private boolean hopperEmptyStopsIndexer = true; // Whether to stop indexer when hopper is empty

    // Shooter configuration (tunable)
    private final LoggedNetworkNumber shooterHeightMeters;
    private final LoggedNetworkNumber shooterOffsetXMeters;
    private final LoggedNetworkNumber shooterOffsetYMeters;
    private final LoggedNetworkNumber flywheelRadiusMeters;
    private final LoggedNetworkNumber launchEfficiency; // Account for energy loss (0.0 - 1.0)

    // Trajectory visualization
    private Pose3d[] lastTrajectory = new Pose3d[0];

    /**
     * Creates a new GamePieceTrajectorySimulation using 2017 FUEL game pieces.
     *
     * @param driveSimulation The swerve drive simulation for robot state
     * @param flywheelVelocityRPMSupplier Supplier for current flywheel velocity in RPM
     * @param hoodAngleDegreesSupplier Supplier for current hood angle in degrees
     */
    public GamePieceTrajectorySimulation(
            SwerveDriveSimulation driveSimulation,
            Supplier<Double> flywheelVelocityRPMSupplier,
            Supplier<Double> hoodAngleDegreesSupplier) {
        this(FUEL_INFO, driveSimulation, flywheelVelocityRPMSupplier, hoodAngleDegreesSupplier);
    }

    /**
     * Creates a new GamePieceTrajectorySimulation.
     *
     * @param gamePieceInfo Info about the game piece being launched (size, mass, etc.)
     * @param driveSimulation The swerve drive simulation for robot state
     * @param flywheelVelocityRPMSupplier Supplier for current flywheel velocity in RPM
     * @param hoodAngleDegreesSupplier Supplier for current hood angle in degrees
     */
    public GamePieceTrajectorySimulation(
            GamePieceOnFieldSimulation.GamePieceInfo gamePieceInfo,
            SwerveDriveSimulation driveSimulation,
            Supplier<Double> flywheelVelocityRPMSupplier,
            Supplier<Double> hoodAngleDegreesSupplier) {
        this(
                gamePieceInfo,
                () -> driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                () -> driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative,
                flywheelVelocityRPMSupplier,
                hoodAngleDegreesSupplier);
    }

    /**
     * Creates a new GamePieceTrajectorySimulation with custom suppliers using 2017 FUEL game pieces.
     *
     * @param robotPositionSupplier Supplier for robot position on field
     * @param robotRotationSupplier Supplier for robot rotation
     * @param chassisSpeedsSupplier Supplier for chassis speeds (field-relative)
     * @param flywheelVelocityRPMSupplier Supplier for flywheel velocity in RPM
     * @param hoodAngleDegreesSupplier Supplier for hood angle in degrees
     */
    public GamePieceTrajectorySimulation(
            Supplier<Translation2d> robotPositionSupplier,
            Supplier<Rotation2d> robotRotationSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            Supplier<Double> flywheelVelocityRPMSupplier,
            Supplier<Double> hoodAngleDegreesSupplier) {
        this(
                FUEL_INFO,
                robotPositionSupplier,
                robotRotationSupplier,
                chassisSpeedsSupplier,
                flywheelVelocityRPMSupplier,
                hoodAngleDegreesSupplier);
    }

    /**
     * Creates a new GamePieceTrajectorySimulation with custom suppliers.
     *
     * @param gamePieceInfo Info about the game piece being launched
     * @param robotPositionSupplier Supplier for robot position on field
     * @param robotRotationSupplier Supplier for robot rotation
     * @param chassisSpeedsSupplier Supplier for chassis speeds (field-relative)
     * @param flywheelVelocityRPMSupplier Supplier for flywheel velocity in RPM
     * @param hoodAngleDegreesSupplier Supplier for hood angle in degrees
     */
    public GamePieceTrajectorySimulation(
            GamePieceOnFieldSimulation.GamePieceInfo gamePieceInfo,
            Supplier<Translation2d> robotPositionSupplier,
            Supplier<Rotation2d> robotRotationSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            Supplier<Double> flywheelVelocityRPMSupplier,
            Supplier<Double> hoodAngleDegreesSupplier) {
        this.gamePieceInfo = gamePieceInfo;
        this.robotPositionSupplier = robotPositionSupplier;
        this.robotRotationSupplier = robotRotationSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
        this.flywheelVelocityRPMSupplier = flywheelVelocityRPMSupplier;
        this.hoodAngleDegreesSupplier = hoodAngleDegreesSupplier;

        // Default shooter configuration (tunable via NetworkTables)
        this.shooterHeightMeters = new LoggedNetworkNumber("Shooter/Sim/HeightMeters", 0.5);
        this.shooterOffsetXMeters = new LoggedNetworkNumber("Shooter/Sim/OffsetXMeters", 0.3);
        this.shooterOffsetYMeters = new LoggedNetworkNumber("Shooter/Sim/OffsetYMeters", 0.0);
        this.flywheelRadiusMeters = new LoggedNetworkNumber("Shooter/Sim/FlywheelRadiusMeters", 0.05);
        this.launchEfficiency = new LoggedNetworkNumber("Shooter/Sim/LaunchEfficiency", 0.85);
    }

    /**
     * Calculates the linear launch velocity from flywheel RPM.
     *
     * @param flywheelRPM Flywheel velocity in RPM
     * @return Launch velocity in meters per second
     */
    public double calculateLaunchVelocityMPS(double flywheelRPM) {
        // Convert RPM to rad/s: RPM * 2π / 60
        double angularVelocityRadPerSec = flywheelRPM * 2.0 * Math.PI / 60.0;

        // Linear velocity = angular velocity * radius * efficiency
        return angularVelocityRadPerSec * flywheelRadiusMeters.get() * launchEfficiency.get();
    }

    /**
     * Gets the current calculated launch velocity based on flywheel state.
     *
     * @return Launch velocity in meters per second
     */
    public double getCurrentLaunchVelocityMPS() {
        return calculateLaunchVelocityMPS(flywheelVelocityRPMSupplier.get());
    }

    /**
     * Gets the shooter position offset from robot center (in robot frame).
     *
     * @return Translation2d representing shooter offset
     */
    public Translation2d getShooterOffset() {
        return new Translation2d(shooterOffsetXMeters.get(), shooterOffsetYMeters.get());
    }

    /**
     * Gets the shooter height from ground.
     *
     * @return Shooter height in meters
     */
    public Distance getShooterHeight() {
        return Meters.of(shooterHeightMeters.get());
    }

    /**
     * Gets the current hood angle.
     *
     * @return Hood angle
     */
    public Angle getHoodAngle() {
        return Degrees.of(hoodAngleDegreesSupplier.get());
    }

    /**
     * Creates and launches a game piece projectile based on current shooter state.
     *
     * @return The launched GamePieceProjectile
     */
    public GamePieceProjectile launchGamePiece() {
        Translation2d robotPosition = robotPositionSupplier.get();
        Rotation2d robotRotation = robotRotationSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

        double launchVelocityMPS = getCurrentLaunchVelocityMPS();
        Distance height = getShooterHeight();
        Angle hoodAngle = getHoodAngle();

        GamePieceProjectile projectile = new GamePieceProjectile(
                gamePieceInfo,
                robotPosition,
                getShooterOffset(),
                chassisSpeeds,
                robotRotation,
                height,
                MetersPerSecond.of(launchVelocityMPS),
                hoodAngle);

        // Configure trajectory callback for visualization
        projectile.withProjectileTrajectoryDisplayCallBack(trajectory -> {
            lastTrajectory = trajectory.toArray(new Pose3d[0]);
            Logger.recordOutput("Shooter/Sim/Trajectory", lastTrajectory);
        });

        // Configure to become game piece on field after touching ground
        projectile.enableBecomesGamePieceOnFieldAfterTouchGround();

        // Add to simulation arena and launch
        SimulatedArena.getInstance().addGamePieceProjectile(projectile);

        // Log launch parameters
        Logger.recordOutput("Shooter/Sim/LaunchVelocityMPS", launchVelocityMPS);
        Logger.recordOutput("Shooter/Sim/LaunchAngleDegrees", hoodAngle.in(Degrees));
        Logger.recordOutput("Shooter/Sim/LaunchHeightMeters", height.in(Meters));
        Logger.recordOutput(
                "Shooter/Sim/LaunchPosition",
                new Pose2d(robotPosition.plus(getShooterOffset().rotateBy(robotRotation)), robotRotation));

        return projectile;
    }

    /**
     * Creates a game piece projectile with custom velocity override.
     *
     * @param launchVelocityMPS Launch velocity in meters per second
     * @return The launched GamePieceProjectile
     */
    public GamePieceProjectile launchGamePiece(double launchVelocityMPS) {
        Translation2d robotPosition = robotPositionSupplier.get();
        Rotation2d robotRotation = robotRotationSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

        Distance height = getShooterHeight();
        Angle hoodAngle = getHoodAngle();

        GamePieceProjectile projectile = new GamePieceProjectile(
                gamePieceInfo,
                robotPosition,
                getShooterOffset(),
                chassisSpeeds,
                robotRotation,
                height,
                MetersPerSecond.of(launchVelocityMPS),
                hoodAngle);

        projectile
                .withProjectileTrajectoryDisplayCallBack(trajectory -> {
                    lastTrajectory = trajectory.toArray(new Pose3d[0]);
                    Logger.recordOutput("Shooter/Sim/Trajectory", lastTrajectory);
                })
                .enableBecomesGamePieceOnFieldAfterTouchGround();

        SimulatedArena.getInstance().addGamePieceProjectile(projectile);

        Logger.recordOutput("Shooter/Sim/LaunchVelocityMPS", launchVelocityMPS);
        Logger.recordOutput("Shooter/Sim/LaunchAngleDegrees", hoodAngle.in(Degrees));

        return projectile;
    }

    /**
     * Creates a game piece projectile with custom velocity and angle override.
     *
     * @param launchVelocityMPS Launch velocity in meters per second
     * @param launchAngleDegrees Launch angle in degrees
     * @return The launched GamePieceProjectile
     */
    public GamePieceProjectile launchGamePiece(double launchVelocityMPS, double launchAngleDegrees) {
        Translation2d robotPosition = robotPositionSupplier.get();
        Rotation2d robotRotation = robotRotationSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

        Distance height = getShooterHeight();

        GamePieceProjectile projectile = new GamePieceProjectile(
                gamePieceInfo,
                robotPosition,
                getShooterOffset(),
                chassisSpeeds,
                robotRotation,
                height,
                MetersPerSecond.of(launchVelocityMPS),
                Degrees.of(launchAngleDegrees));

        projectile
                .withProjectileTrajectoryDisplayCallBack(trajectory -> {
                    lastTrajectory = trajectory.toArray(new Pose3d[0]);
                    Logger.recordOutput("Shooter/Sim/Trajectory", lastTrajectory);
                })
                .enableBecomesGamePieceOnFieldAfterTouchGround();

        SimulatedArena.getInstance().addGamePieceProjectile(projectile);

        Logger.recordOutput("Shooter/Sim/LaunchVelocityMPS", launchVelocityMPS);
        Logger.recordOutput("Shooter/Sim/LaunchAngleDegrees", launchAngleDegrees);

        return projectile;
    }

    /**
     * Calculates and previews trajectory without actually launching. Useful for aiming assistance.
     *
     * @return Array of Pose3d representing predicted trajectory
     */
    public Pose3d[] previewTrajectory() {
        double launchVelocityMPS = getCurrentLaunchVelocityMPS();
        double hoodAngleRad = getHoodAngle().in(Radians);
        double heightM = shooterHeightMeters.get();

        Translation2d robotPosition = robotPositionSupplier.get();
        Rotation2d robotRotation = robotRotationSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

        // Calculate initial velocity components
        double horizontalVelocity = launchVelocityMPS * Math.cos(hoodAngleRad);
        double verticalVelocity = launchVelocityMPS * Math.sin(hoodAngleRad);

        // Add chassis velocity contribution
        Translation2d shooterOffset = getShooterOffset();
        Translation2d chassisVelocity =
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        Translation2d shooterRotationalVelocity = shooterOffset
                .rotateBy(robotRotation)
                .rotateBy(Rotation2d.fromDegrees(90))
                .times(chassisSpeeds.omegaRadiansPerSecond);

        Translation2d totalHorizontalVelocity = chassisVelocity
                .plus(shooterRotationalVelocity)
                .plus(new Translation2d(horizontalVelocity, robotRotation));

        // Calculate initial position
        Translation2d launchPosition = robotPosition.plus(shooterOffset.rotateBy(robotRotation));

        // Generate trajectory preview (100 steps, 0.02s each = 2 seconds)
        Pose3d[] trajectory = new Pose3d[100];
        double dt = 0.02;
        double gravity = 11.0; // Maple-sim uses adjusted gravity

        for (int i = 0; i < trajectory.length; i++) {
            double t = i * dt;
            double x = launchPosition.getX() + totalHorizontalVelocity.getX() * t;
            double y = launchPosition.getY() + totalHorizontalVelocity.getY() * t;
            double z = heightM + verticalVelocity * t - 0.5 * gravity * t * t;

            if (z < 0) {
                // Ground hit, truncate trajectory
                Pose3d[] truncated = new Pose3d[i];
                System.arraycopy(trajectory, 0, truncated, 0, i);
                lastTrajectory = truncated;
                Logger.recordOutput("Shooter/Sim/PreviewTrajectory", lastTrajectory);
                return truncated;
            }

            trajectory[i] = new Pose3d(x, y, z, new Rotation3d());
        }

        lastTrajectory = trajectory;
        Logger.recordOutput("Shooter/Sim/PreviewTrajectory", lastTrajectory);
        return trajectory;
    }

    /**
     * Gets the last calculated/launched trajectory.
     *
     * @return Array of Pose3d representing trajectory
     */
    public Pose3d[] getLastTrajectory() {
        return lastTrajectory;
    }

    /**
     * Calculates the predicted landing position of the game piece.
     *
     * @return Translation3d of predicted landing position, or null if trajectory goes out of bounds
     */
    public Translation3d getPredictedLandingPosition() {
        Pose3d[] trajectory = previewTrajectory();
        if (trajectory.length > 0) {
            return trajectory[trajectory.length - 1].getTranslation();
        }
        return null;
    }

    /**
     * Sets the target position for hit detection. Configure this for scoring targets.
     *
     * @param targetPosition 3D position of the target
     * @param tolerance Tolerance for hit detection in each axis
     * @param onHit Callback when target is hit
     * @return A configured projectile (not launched)
     */
    public GamePieceProjectile createTargetedProjectile(
            Supplier<Translation3d> targetPosition, Translation3d tolerance, Runnable onHit) {
        Translation2d robotPosition = robotPositionSupplier.get();
        Rotation2d robotRotation = robotRotationSupplier.get();
        ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

        GamePieceProjectile projectile = new GamePieceProjectile(
                gamePieceInfo,
                robotPosition,
                getShooterOffset(),
                chassisSpeeds,
                robotRotation,
                getShooterHeight(),
                MetersPerSecond.of(getCurrentLaunchVelocityMPS()),
                getHoodAngle());

        projectile
                .withTargetPosition(targetPosition)
                .withTargetTolerance(tolerance)
                .withHitTargetCallBack(onHit)
                .withProjectileTrajectoryDisplayCallBack(
                        trajectoryHit -> {
                            lastTrajectory = trajectoryHit.toArray(new Pose3d[0]);
                            Logger.recordOutput("Shooter/Sim/Trajectory", lastTrajectory);
                        },
                        trajectoryMiss -> {
                            lastTrajectory = trajectoryMiss.toArray(new Pose3d[0]);
                            Logger.recordOutput("Shooter/Sim/TrajectoryMiss", lastTrajectory);
                        });

        return projectile;
    }

    // ==================== Auto-Fire Simulation ====================

    /**
     * Configures the indexer running supplier for automatic firing simulation. When the indexer is running, the
     * simulation will automatically launch game pieces at the configured interval.
     *
     * @param indexerRunningSupplier Supplier that returns true when indexer is feeding balls
     */
    public void setIndexerRunningSupplier(BooleanSupplier indexerRunningSupplier) {
        this.indexerRunningSupplier = indexerRunningSupplier;
    }

    /**
     * Sets the interval between automatic game piece launches when indexer is running.
     *
     * @param intervalSeconds Time between launches in seconds (default 1.0)
     */
    public void setAutoFireInterval(double intervalSeconds) {
        this.autoFireIntervalSeconds = intervalSeconds;
    }

    /**
     * Enables or disables automatic firing when indexer is running.
     *
     * @param enabled True to enable auto-fire
     */
    public void setAutoFireEnabled(boolean enabled) {
        this.autoFireEnabled = enabled;
        if (enabled) {
            autoFireTimer.restart();
        } else {
            autoFireTimer.stop();
        }
    }

    /**
     * Gets whether auto-fire is currently enabled.
     *
     * @return True if auto-fire is enabled
     */
    public boolean isAutoFireEnabled() {
        return autoFireEnabled;
    }

    /**
     * Gets the total number of game pieces launched this session.
     *
     * @return Number of game pieces launched
     */
    public int getGamePiecesLaunched() {
        return gamePiecesLaunched;
    }

    /** Resets the game pieces launched counter. */
    public void resetGamePiecesLaunched() {
        gamePiecesLaunched = 0;
    }

    /**
     * Gets the current number of balls in the hopper.
     *
     * @return Number of balls remaining
     */
    public int getBallsInHopper() {
        return (int) ballsInHopper.get();
    }

    /**
     * Sets the number of balls in the hopper.
     *
     * @param count Number of balls
     */
    public void setBallsInHopper(int count) {
        ballsInHopper.set(Math.max(0, count));
    }

    /**
     * Adds balls to the hopper (e.g., when intaking).
     *
     * @param count Number of balls to add
     */
    public void addBalls(int count) {
        setBallsInHopper(getBallsInHopper() + count);
    }

    /**
     * Checks if the hopper has any balls.
     *
     * @return true if there is at least one ball in the hopper
     */
    public boolean hasBalls() {
        return getBallsInHopper() > 0;
    }

    /**
     * Checks if the hopper is empty.
     *
     * @return true if the hopper is empty
     */
    public boolean isEmpty() {
        return getBallsInHopper() <= 0;
    }

    /**
     * Sets whether an empty hopper should stop the indexer from running.
     *
     * @param enabled true to stop indexer when hopper is empty
     */
    public void setHopperEmptyStopsIndexer(boolean enabled) {
        this.hopperEmptyStopsIndexer = enabled;
    }

    /**
     * Returns whether the indexer should be allowed to run based on hopper state. This can be used as a condition in
     * commands or triggers.
     *
     * @return true if indexer should be allowed to run (hopper has balls or feature disabled)
     */
    public boolean shouldIndexerRun() {
        if (!hopperEmptyStopsIndexer) {
            return true; // Feature disabled, always allow
        }
        return hasBalls();
    }

    /**
     * Updates the auto-fire simulation. Call this method periodically (e.g., in a subsystem's periodic method or robot
     * periodic). When auto-fire is enabled and the indexer is running, this will launch a game piece at the configured
     * interval. Will not fire if hopper is empty and hopperEmptyStopsIndexer is enabled.
     */
    public void updateAutoFire() {
        int currentBalls = getBallsInHopper();

        Logger.recordOutput("Shooter/Sim/AutoFireEnabled", autoFireEnabled);
        Logger.recordOutput("Shooter/Sim/IndexerRunning", indexerRunningSupplier.getAsBoolean());
        Logger.recordOutput("Shooter/Sim/GamePiecesLaunched", gamePiecesLaunched);
        Logger.recordOutput("Shooter/Sim/BallsRemaining", currentBalls);
        Logger.recordOutput("Shooter/Sim/HopperEmpty", currentBalls <= 0);
        Logger.recordOutput("Shooter/Sim/IndexerAllowed", shouldIndexerRun());

        if (!autoFireEnabled) {
            return;
        }

        // Check if we should allow firing based on hopper state
        if (hopperEmptyStopsIndexer && currentBalls <= 0) {
            // Hopper is empty and we're configured to stop - don't fire
            autoFireTimer.restart(); // Keep timer fresh for when balls are added
            return;
        }

        boolean indexerRunning = indexerRunningSupplier.getAsBoolean();

        if (indexerRunning) {
            // Check if enough time has passed since last launch
            if (autoFireTimer.hasElapsed(autoFireIntervalSeconds)) {
                // Launch a game piece and decrement ball count
                launchGamePiece();
                gamePiecesLaunched++;
                setBallsInHopper(currentBalls - 1);
                autoFireTimer.restart();

                Logger.recordOutput("Shooter/Sim/LastLaunchTime", Timer.getFPGATimestamp());
            }
        } else {
            // Reset timer when indexer stops so next ball fires immediately when indexer starts again
            autoFireTimer.restart();
        }
    }

    /**
     * Convenience method to enable auto-fire with the given indexer supplier. Equivalent to calling
     * setIndexerRunningSupplier() followed by setAutoFireEnabled(true).
     *
     * @param indexerRunningSupplier Supplier that returns true when indexer is feeding balls
     */
    public void enableAutoFire(BooleanSupplier indexerRunningSupplier) {
        setIndexerRunningSupplier(indexerRunningSupplier);
        setAutoFireEnabled(true);
    }

    /**
     * Convenience method to enable auto-fire with custom interval.
     *
     * @param indexerRunningSupplier Supplier that returns true when indexer is feeding balls
     * @param intervalSeconds Time between launches in seconds
     */
    public void enableAutoFire(BooleanSupplier indexerRunningSupplier, double intervalSeconds) {
        setIndexerRunningSupplier(indexerRunningSupplier);
        setAutoFireInterval(intervalSeconds);
        setAutoFireEnabled(true);
    }
}
