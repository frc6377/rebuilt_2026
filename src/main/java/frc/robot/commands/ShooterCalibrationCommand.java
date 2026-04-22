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

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.BaseShooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.superstructure.GamePieceTrajectorySimulation;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

/**
 * A calibration command that systematically tests different shooting parameters from various positions to find values
 * that successfully score in the hub.
 *
 * <p>ONLY FOR USE IN SIMULATION!
 *
 * <p>The command will:
 *
 * <ol>
 *   <li>Move to each test position around the hub
 *   <li>Try different RPM combinations
 *   <li>Fire a test shot and wait to see if it scores
 *   <li>Record successful combinations
 *   <li>Move to the next position and repeat
 * </ol>
 */
public class ShooterCalibrationCommand extends Command {
    /** Stores a successful shot configuration. */
    public record ShotConfiguration(
            Distance distance, Angle hoodAngle, AngularVelocity flywheelVelocity, boolean scored) {
        @Override
        public @NotNull String toString() {
            return String.format(
                    "%.2fm -> Hood: %.1f°, RPM: %.0f %s",
                    this.distance.in(Meters), this.hoodAngle.in(Degrees), this.flywheelVelocity.in(RPM), this.scored ? "✓" : "✗");
        }
    }

    // Subsystems
    private final BaseShooter shooter;
    private final GamePieceTrajectorySimulation trajectorySim;
    private final SwerveDriveSimulation driveSim;
    private final Consumer<Pose2d> poseResetter;
    // Test parameters
    private final Distance[] testDistances;
    private final AngularVelocity lowerBound;
    private final AngularVelocity upperBound;
    private AngularVelocity currentVelocity;
    private final double speedInterations = 8;

    // State tracking
    private int currentDistanceIndex = 0;
    private int currentRPMIndex = 0;
    private @NotNull CalibrationState state = CalibrationState.SETUP_POSITION;
    private final Timer stateTimer = new Timer();
    private GamePieceProjectile currentShot; // Track the current shot for trajectory analysis
    // Results
    private final List<ShotConfiguration> results = new ArrayList<>();
    private final List<ShotConfiguration> successfulShots = new ArrayList<>();

    // Timing constants
    private static final double SETTLE_TIME = 0.3; // Time to wait for mechanisms to settle (reduced for speed)
    private static final double SHOT_TRAVEL_TIME = 3.0; // Max time to wait for ball (fallback)

    // Hub scoring detection
    private Translation2d hubPosition;
    private static final double HUB_RADIUS =
            FieldConstants.Hub.innerWidth / 2.0 / 6; // Use quarter the radius to be conservative
    private static final double HUB_HEIGHT = FieldConstants.Hub.height;

    // Shot result tracking (for early termination)
    private ShotResult lastShotResult = ShotResult.PENDING;
    private AngularVelocity tempLowerBound;
    private AngularVelocity tempUpperBound;

    private enum ShotResult {
        PENDING, // Still tracking
        SCORED, // Ball entered hub from above
        MISSED_LOW, // Ball is descending below hub height
        MISSED_FAR, // Ball passed the hub horizontally
        MISSED_SHORT, // Ball landed before reaching hub
        TIMEOUT // Max time exceeded
    }

    private enum CalibrationState {
        SETUP_POSITION,
        WAIT_FOR_SETTLE,
        FIRE_SHOT,
        WAIT_FOR_RESULT,
        NEXT_CONFIGURATION,
        COMPLETE
    }

    /**
     * Creates a new ShooterCalibrationCommand with default test ranges.
     *
     * @param shooter Shooter subsystem
     * @param trajectorySim Trajectory simulation
     * @param driveSim Drive simulation for teleporting robot
     * @param poseResetter Consumer to reset odometry pose
     */
    public ShooterCalibrationCommand(
            BaseShooter shooter,
            GamePieceTrajectorySimulation trajectorySim,
            SwerveDriveSimulation driveSim,
            Consumer<Pose2d> poseResetter) {
        this(
                shooter,
                trajectorySim,
                driveSim,
                poseResetter,
                // Default test distances: 2m to 6m in 0.5m increments
                new Distance[] {
                    Meters.of(2.0),
                    Meters.of(2.5),
                    Meters.of(3.0),
                    Meters.of(3.5),
                    Meters.of(4.0),
                    Meters.of(4.5),
                    Meters.of(5.0),
                    Meters.of(5.5),
                    Meters.of(6.0)
                },
                RPM.of(2000),
                RPM.of(6000));
    }

    public ShooterCalibrationCommand(@NotNull Superstructure superstructure, @NotNull SwerveDriveSimulation drive) {
        this(
                superstructure.getLeftShooter(),
                superstructure.getGamePieceTrajectorySimulation(),
                drive,
                drive::setSimulationWorldPose);
    }
    /**
     * Creates a new ShooterCalibrationCommand with custom test ranges.
     *
     * @param shooter Shooter subsystem
     * @param trajectorySim Trajectory simulation
     * @param driveSim Drive simulation for teleporting robot
     * @param poseResetter Consumer to reset odometry pose
     * @param testDistances Array of distances to test from
     * @param lowerBound Minimum flywheel velocity to test
     * @param upperBound Maximum flywheel velocity to test
     */
    public ShooterCalibrationCommand(
            BaseShooter shooter,
            GamePieceTrajectorySimulation trajectorySim,
            SwerveDriveSimulation driveSim,
            Consumer<Pose2d> poseResetter,
            Distance[] testDistances,
            AngularVelocity lowerBound,
            AngularVelocity upperBound) {

        this.shooter = shooter;
        this.trajectorySim = trajectorySim;
        this.driveSim = driveSim;
        this.poseResetter = poseResetter;
        this.testDistances = testDistances;
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
        this.tempLowerBound = lowerBound;
        this.tempUpperBound = upperBound;

        this.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        this.hubPosition = FieldConstants.getHubPosition();
        this.currentDistanceIndex = 0;
        this.currentRPMIndex = 0;
        this.state = CalibrationState.SETUP_POSITION;
        this.results.clear();
        this.successfulShots.clear();
        this.stateTimer.restart();

        // Ensure we have balls to shoot
        this.trajectorySim.setBallsInHopper(1000); // Lots of balls for testing

        Logger.recordOutput("Calibration/Status", "Starting calibration...");
        Logger.recordOutput("Calibration/TotalTests", this.testDistances.length * this.speedInterations);
    }

    @Override
    public void execute() {
        // Log current state
        Logger.recordOutput("Calibration/State", this.state.toString());
        Logger.recordOutput("Calibration/DistanceIndex", this.currentDistanceIndex);
        Logger.recordOutput("Calibration/RPMIndex", this.currentRPMIndex);
        Logger.recordOutput("Calibration/SuccessfulShots", this.successfulShots.size());

        switch (this.state) {
            case SETUP_POSITION -> {

                // Teleport robot to test position facing the hub
                Distance currentDistance = this.testDistances[this.currentDistanceIndex];
                Pose2d testPose = this.calculateTestPose(currentDistance);
                this.lastShotResult = this.getShotResult();
                // Teleport the simulated robot
                this.driveSim.setSimulationWorldPose(testPose);
                this.poseResetter.accept(testPose);

                // Set flywheel speed
                if (ShotResult.SCORED == lastShotResult) {
                    // If the last shot scored, we can try a slightly higher velocity
                    this.currentRPMIndex = 100;
                } else if (ShotResult.MISSED_SHORT == lastShotResult || ShotResult.MISSED_LOW == lastShotResult) {
                    // If the last shot was short or low, we need to increase velocity
                    this.tempLowerBound = this.currentVelocity;
                    this.currentVelocity = this.tempLowerBound.plus(this.tempUpperBound).times(0.5);
                } else if (ShotResult.MISSED_FAR == lastShotResult) {
                    // If the last shot was far, we need to decrease velocity
                    this.tempUpperBound = this.currentVelocity;
                    this.currentVelocity = this.tempLowerBound.plus(this.tempUpperBound).times(0.5);
                } else {
                    // If we don't have a result yet, just try the midpoint
                    this.currentVelocity = this.tempLowerBound.plus(this.tempUpperBound).times(0.5);
                }
                this.shooter.setFlywheelVelocity(this.currentVelocity);

                Logger.recordOutput("Calibration/TestDistance", currentDistance.in(Meters));
                Logger.recordOutput("Calibration/TestRPM", this.currentVelocity.in(RPM));
                Logger.recordOutput("Calibration/LowerBoundRPM", this.lowerBound.in(RPM));
                Logger.recordOutput("Calibration/UpperBoundRPM", this.upperBound.in(RPM));
                this.stateTimer.restart();
                this.state = CalibrationState.WAIT_FOR_SETTLE;
            }

            case WAIT_FOR_SETTLE -> {
                // Wait for flywheel to reach setpoint
                if (this.stateTimer.hasElapsed(SETTLE_TIME)) {
                    double actualRPM = this.shooter.getFlywheelVelocity().in(RPM);
                    if (0.05 > Math.abs(actualRPM - currentVelocity.in(RotationsPerSecond))
                            / currentVelocity.in(RotationsPerSecond)) {
                        this.state = CalibrationState.FIRE_SHOT;
                    } else if (this.stateTimer.hasElapsed(SETTLE_TIME * 3)) {
                        // Timeout - proceed anyway
                        this.state = CalibrationState.FIRE_SHOT;
                    }
                }
            }

            case FIRE_SHOT -> {
                // Fire a single shot
                this.currentShot = this.trajectorySim.launchGamePiece();

                this.stateTimer.restart();
                this.state = CalibrationState.WAIT_FOR_RESULT;
            }

            case WAIT_FOR_RESULT -> {
                // Wait for ball to travel and check if it scored
                Logger.recordOutput("Calibration/Current Position", this.currentShot.getPose3d());
                if (this.stateTimer.hasElapsed(SHOT_TRAVEL_TIME)) {
                    // Determine if shot was successful by checking trajectory endpoint
                    boolean scored = this.checkIfScored();

                    // Record result
                    ShotConfiguration config = new ShotConfiguration(
                            this.testDistances[this.currentDistanceIndex],
                            ShooterConstants.kFixedHoodAngle,
                            this.currentVelocity,
                            scored);
                    this.results.add(config);

                    if (scored) {
                        this.tempLowerBound = this.lowerBound;
                        this.tempUpperBound = this.upperBound;
                        this.successfulShots.add(config);
                        Logger.recordOutput("Calibration/LastResult", "SUCCESS: " + config + this.lastShotResult);
                    } else {
                        Logger.recordOutput("Calibration/LastResult", "MISS: " + config + this.lastShotResult);
                    }

                    this.state = CalibrationState.NEXT_CONFIGURATION;
                }
            }

            case NEXT_CONFIGURATION -> {
                // Move to next configuration
                this.currentRPMIndex++;
                if (this.speedInterations <= currentRPMIndex) {
                    this.currentRPMIndex = 0;
                    this.currentDistanceIndex++;
                    if (this.currentDistanceIndex >= this.testDistances.length) {
                        // All tests complete
                        this.state = CalibrationState.COMPLETE;
                        return;
                    }
                }
                this.state = CalibrationState.SETUP_POSITION;
            }

            case COMPLETE -> {
                // Log final results
                this.logResults();
            }
        }
    }

    /**
     * Calculate the test pose for a given distance from the hub.
     *
     * @param distance Distance from hub
     * @return Pose2d positioned that distance from hub, facing the hub
     */
    private @NotNull Pose2d calculateTestPose(@NotNull Distance distance) {
        // Position robot directly in front of hub (towards blue alliance wall)
        double x = this.hubPosition.getX() - distance.in(Meters);
        double y = this.hubPosition.getY();

        // Face towards the hub
        Rotation2d rotation = new Rotation2d(0); // Facing positive X (towards hub)

        return new Pose2d(x, y, rotation);
    }

    /**
     * Check if the last shot scored in the hub. Uses trajectory endpoint analysis. The ball must enter from above
     * (descending) to count as a score.
     *
     * @return true if the shot likely scored
     */
    private boolean checkIfScored() {
        // Get the last trajectory from the simulation
        Pose3d[] trajectory = this.trajectorySim.getLastTrajectory();
        if (null == trajectory || 2 > trajectory.length) {
            return false;
        }

        // Check the trajectory points to see if the ball passes through the hub opening from above
        for (int i = 1; i < trajectory.length; i++) {
            Pose3d prevPoint = trajectory[i - 1];
            Pose3d point = trajectory[i];

            double dx = point.getX() - this.hubPosition.getX();
            double dy = point.getY() - this.hubPosition.getY();
            double horizontalDistance = Math.sqrt(dx * dx + dy * dy);
            double height = point.getZ();
            double prevHeight = prevPoint.getZ();

            // Check if ball is descending (coming from above)
            boolean isDescending = height < prevHeight;

            // Check if point is within hub opening and ball is descending
            // Ball scores if:
            // 1. It's within the hub radius horizontally
            // 2. It's at the right height (hub opening)
            // 3. It's descending (entering from above, not below)
            if (horizontalDistance < HUB_RADIUS
                    && height > HUB_HEIGHT - 0.3
                    && height < HUB_HEIGHT + 0.5
                    && isDescending) {
                return true;
            }
        }

        return false;
    }

    private ShotResult getShotResult() {
        Pose3d[] trajectory = this.trajectorySim.getLastTrajectory();
        if (null == trajectory || 2 > trajectory.length) {
            return ShotResult.TIMEOUT;
        }

        for (int i = 1; i < trajectory.length; i++) {
            Pose3d prevPoint = trajectory[i - 1];
            Pose3d point = trajectory[i];

            double dx = point.getX() - this.hubPosition.getX();
            double dy = point.getY() - this.hubPosition.getY();
            double horizontalDistance = Math.sqrt(dx * dx + dy * dy);
            double height = point.getZ();
            double prevHeight = prevPoint.getZ();

            boolean isDescending = height < prevHeight;

            if (horizontalDistance < HUB_RADIUS
                    && height > HUB_HEIGHT - 0.3
                    && height < HUB_HEIGHT + 0.5
                    && isDescending) {
                return ShotResult.SCORED;
            }

            // Check if ball has passed the hub horizontally
            if (point.getX() > this.hubPosition.getX() + HUB_RADIUS) {
                return ShotResult.MISSED_FAR;
            }

            // Check if ball has landed before reaching the hub
            if (0.1 > height && point.getX() < this.hubPosition.getX() - HUB_RADIUS) {
                return ShotResult.MISSED_SHORT;
            }

            // Check if ball is descending below hub height without scoring
            if (height < HUB_HEIGHT - 0.3 && isDescending) {
                return ShotResult.MISSED_LOW;
            }
        }
        Timer.delay(1);
        return this.getShotResult();
    }
    /** Log the calibration results. */
    private void logResults() {
        Logger.recordOutput("Calibration/Status", "COMPLETE");
        Logger.recordOutput("Calibration/TotalTests", this.results.size());
        Logger.recordOutput("Calibration/SuccessfulShots", this.successfulShots.size());

        // Build results summary
        StringBuilder summary = new StringBuilder();
        summary.append("=== CALIBRATION RESULTS ===\n");
        summary.append(String.format("Total tests: %d\n", this.results.size()));
        summary.append(String.format("Successful: %d\n", this.successfulShots.size()));
        summary.append("\n--- Successful Configurations ---\n");

        for (ShotConfiguration config : this.successfulShots) {
            summary.append(config.toString()).append("\n");
        }

        Logger.recordOutput("Calibration/Summary", summary.toString());

        // Also log as individual entries for easy filtering
        for (int i = 0; i < this.successfulShots.size(); i++) {
            ShotConfiguration config = this.successfulShots.get(i);
            Logger.recordOutput(
                    "Calibration/Success/" + i + "/Distance", config.distance().in(Meters));
            Logger.recordOutput(
                    "Calibration/Success/" + i + "/Angle", config.hoodAngle().in(Degrees));
            Logger.recordOutput(
                    "Calibration/Success/" + i + "/RPM",
                    config.flywheelVelocity().in(RPM));
        }

        // Print to console as well
        System.out.println(summary);
    }

    @Override
    public boolean isFinished() {
        return CalibrationState.COMPLETE == state;
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.stop();
        if (interrupted) {
            Logger.recordOutput("Calibration/Status", "INTERRUPTED");
        } else {
            this.logResults();
        }
    }

    /**
     * Get all successful shot configurations found during calibration.
     *
     * @return List of successful configurations
     */
    public @NotNull List<ShotConfiguration> getSuccessfulShots() {
        return new ArrayList<>(this.successfulShots);
    }

    /**
     * Get all test results.
     *
     * @return List of all configurations tested
     */
    public @NotNull List<ShotConfiguration> getAllResults() {
        return new ArrayList<>(this.results);
    }
}
