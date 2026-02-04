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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.GamePieceTrajectorySimulation;
import frc.robot.subsystems.shooter.Shooter;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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
 *   <li>Try different hood angle and RPM combinations
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
        public String toString() {
            return String.format(
                    "%.2fm -> Hood: %.1f°, RPM: %.0f %s",
                    distance.in(Meters), hoodAngle.in(Degrees), flywheelVelocity.in(RPM), scored ? "✓" : "✗");
        }
    }

    // Subsystems
    private final Hood hood;
    private final Shooter shooter;
    private final GamePieceTrajectorySimulation trajectorySim;
    private final SwerveDriveSimulation driveSim;
    private final Consumer<Pose2d> poseResetter;

    // Test parameters
    private final Distance[] testDistances;
    private final Angle[] testAngles;
    private final AngularVelocity[] testRPMs;

    // State tracking
    private int currentDistanceIndex = 0;
    private int currentAngleIndex = 0;
    private int currentRPMIndex = 0;
    private CalibrationState state = CalibrationState.SETUP_POSITION;
    private final Timer stateTimer = new Timer();

    // Results
    private final List<ShotConfiguration> results = new ArrayList<>();
    private final List<ShotConfiguration> successfulShots = new ArrayList<>();

    // Timing constants
    private static final double SETTLE_TIME = 0.3; // Time to wait for mechanisms to settle (reduced for speed)
    private static final double SHOT_TRAVEL_TIME = 3.0; // Max time to wait for ball (fallback)
    private static final double MIN_CHECK_INTERVAL = 0.02; // Check trajectory every 20ms

    // Hub scoring detection
    private Translation2d hubPosition;
    private static final double HUB_RADIUS = FieldConstants.HUB_OPENING_DIAMETER.in(Meters) / 2.0;
    private static final double HUB_HEIGHT = FieldConstants.HUB_OPENING_HEIGHT.in(Meters);

    // Shot result tracking (for early termination)
    private ShotResult lastShotResult = ShotResult.PENDING;

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
     * @param hood Hood subsystem
     * @param shooter Shooter subsystem
     * @param trajectorySim Trajectory simulation
     * @param driveSim Drive simulation for teleporting robot
     * @param poseResetter Consumer to reset odometry pose
     */
    public ShooterCalibrationCommand(
            Hood hood,
            Shooter shooter,
            GamePieceTrajectorySimulation trajectorySim,
            SwerveDriveSimulation driveSim,
            Consumer<Pose2d> poseResetter) {
        this(
                hood,
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
                // Default test angles: 30 to 70 degrees in 5 degree increments
                new Angle[] {
                    Degrees.of(40),
                    Degrees.of(45),
                    Degrees.of(50),
                    Degrees.of(55),
                    Degrees.of(60),
                    Degrees.of(65),
                    Degrees.of(70),
                    Degrees.of(75),
                    Degrees.of(80)
                },
                // Default test RPMs: 2000 to 5000 in 500 RPM increments
                new AngularVelocity[] {
                    RPM.of(2000), RPM.of(2500), RPM.of(3000), RPM.of(3500), RPM.of(4000), RPM.of(4500), RPM.of(5000)
                });
    }

    /**
     * Creates a new ShooterCalibrationCommand with custom test ranges.
     *
     * @param hood Hood subsystem
     * @param shooter Shooter subsystem
     * @param trajectorySim Trajectory simulation
     * @param driveSim Drive simulation for teleporting robot
     * @param poseResetter Consumer to reset odometry pose
     * @param testDistances Array of distances to test from
     * @param testAngles Array of hood angles to test
     * @param testRPMs Array of flywheel RPMs to test
     */
    public ShooterCalibrationCommand(
            Hood hood,
            Shooter shooter,
            GamePieceTrajectorySimulation trajectorySim,
            SwerveDriveSimulation driveSim,
            Consumer<Pose2d> poseResetter,
            Distance[] testDistances,
            Angle[] testAngles,
            AngularVelocity[] testRPMs) {
        this.hood = hood;
        this.shooter = shooter;
        this.trajectorySim = trajectorySim;
        this.driveSim = driveSim;
        this.poseResetter = poseResetter;
        this.testDistances = testDistances;
        this.testAngles = testAngles;
        this.testRPMs = testRPMs;

        addRequirements(hood, shooter);
    }

    @Override
    public void initialize() {
        hubPosition = FieldConstants.getHubPosition();
        currentDistanceIndex = 0;
        currentAngleIndex = 0;
        currentRPMIndex = 0;
        state = CalibrationState.SETUP_POSITION;
        results.clear();
        successfulShots.clear();
        stateTimer.restart();

        // Ensure we have balls to shoot
        trajectorySim.setBallsInHopper(1000); // Lots of balls for testing

        Logger.recordOutput("Calibration/Status", "Starting calibration...");
        Logger.recordOutput("Calibration/TotalTests", testDistances.length * testAngles.length * testRPMs.length);
    }

    @Override
    public void execute() {
        // Log current state
        Logger.recordOutput("Calibration/State", state.toString());
        Logger.recordOutput("Calibration/DistanceIndex", currentDistanceIndex);
        Logger.recordOutput("Calibration/AngleIndex", currentAngleIndex);
        Logger.recordOutput("Calibration/RPMIndex", currentRPMIndex);
        Logger.recordOutput("Calibration/SuccessfulShots", successfulShots.size());

        switch (state) {
            case SETUP_POSITION -> {
                // Teleport robot to test position facing the hub
                Distance currentDistance = testDistances[currentDistanceIndex];
                Pose2d testPose = calculateTestPose(currentDistance);

                // Teleport the simulated robot
                driveSim.setSimulationWorldPose(testPose);
                poseResetter.accept(testPose);

                // Set hood angle and flywheel speed
                Angle currentAngle = testAngles[currentAngleIndex];
                AngularVelocity currentRPM = testRPMs[currentRPMIndex];

                hood.setAngle(currentAngle.in(Degrees));
                shooter.setFlywheelVelocity(currentRPM.in(RPM));

                Logger.recordOutput("Calibration/TestDistance", currentDistance.in(Meters));
                Logger.recordOutput("Calibration/TestAngle", currentAngle.in(Degrees));
                Logger.recordOutput("Calibration/TestRPM", currentRPM.in(RPM));

                stateTimer.restart();
                state = CalibrationState.WAIT_FOR_SETTLE;
            }

            case WAIT_FOR_SETTLE -> {
                // Wait for hood and flywheel to reach setpoint
                if (stateTimer.hasElapsed(SETTLE_TIME)) {
                    // Check if flywheel is up to speed (within 5%)
                    double targetRPM = testRPMs[currentRPMIndex].in(RPM);
                    double actualRPM = (shooter.getLeftFlywheelVelocity() + shooter.getRightFlywheelVelocity()) / 2.0;
                    if (Math.abs(actualRPM - targetRPM) / targetRPM < 0.05) {
                        state = CalibrationState.FIRE_SHOT;
                    } else if (stateTimer.hasElapsed(SETTLE_TIME * 3)) {
                        // Timeout - proceed anyway
                        state = CalibrationState.FIRE_SHOT;
                    }
                }
            }

            case FIRE_SHOT -> {
                // Fire a single shot
                trajectorySim.launchGamePiece();

                stateTimer.restart();
                state = CalibrationState.WAIT_FOR_RESULT;
            }

            case WAIT_FOR_RESULT -> {
                // Wait for ball to travel and check if it scored
                if (stateTimer.hasElapsed(SHOT_TRAVEL_TIME)) {
                    // Determine if shot was successful by checking trajectory endpoint
                    // For now, we use a simple heuristic based on the last trajectory
                    boolean scored = checkIfScored();

                    // Record result
                    ShotConfiguration config = new ShotConfiguration(
                            testDistances[currentDistanceIndex],
                            testAngles[currentAngleIndex],
                            testRPMs[currentRPMIndex],
                            scored);
                    results.add(config);

                    if (scored) {
                        successfulShots.add(config);
                        Logger.recordOutput("Calibration/LastResult", "SUCCESS: " + config.toString());
                    } else {
                        Logger.recordOutput("Calibration/LastResult", "MISS: " + config.toString());
                    }

                    state = CalibrationState.NEXT_CONFIGURATION;
                }
            }

            case NEXT_CONFIGURATION -> {
                // Move to next configuration
                currentRPMIndex++;
                if (currentRPMIndex >= testRPMs.length) {
                    currentRPMIndex = 0;
                    currentAngleIndex++;
                    if (currentAngleIndex >= testAngles.length) {
                        currentAngleIndex = 0;
                        currentDistanceIndex++;
                        if (currentDistanceIndex >= testDistances.length) {
                            // All tests complete
                            state = CalibrationState.COMPLETE;
                            return;
                        }
                    }
                }
                state = CalibrationState.SETUP_POSITION;
            }

            case COMPLETE -> {
                // Log final results
                logResults();
            }
        }
    }

    /**
     * Calculate the test pose for a given distance from the hub.
     *
     * @param distance Distance from hub
     * @return Pose2d positioned that distance from hub, facing the hub
     */
    private Pose2d calculateTestPose(Distance distance) {
        // Position robot directly in front of hub (towards blue alliance wall)
        double x = hubPosition.getX() - distance.in(Meters);
        double y = hubPosition.getY();

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
        Pose3d[] trajectory = trajectorySim.getLastTrajectory();
        if (trajectory == null || trajectory.length < 2) {
            return false;
        }

        // Check the trajectory points to see if the ball passes through the hub opening from above
        for (int i = 1; i < trajectory.length; i++) {
            Pose3d prevPoint = trajectory[i - 1];
            Pose3d point = trajectory[i];

            double dx = point.getX() - hubPosition.getX();
            double dy = point.getY() - hubPosition.getY();
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

    /** Log the calibration results. */
    private void logResults() {
        Logger.recordOutput("Calibration/Status", "COMPLETE");
        Logger.recordOutput("Calibration/TotalTests", results.size());
        Logger.recordOutput("Calibration/SuccessfulShots", successfulShots.size());

        // Build results summary
        StringBuilder summary = new StringBuilder();
        summary.append("=== CALIBRATION RESULTS ===\n");
        summary.append(String.format("Total tests: %d\n", results.size()));
        summary.append(String.format("Successful: %d\n", successfulShots.size()));
        summary.append("\n--- Successful Configurations ---\n");

        for (ShotConfiguration config : successfulShots) {
            summary.append(config.toString()).append("\n");
        }

        Logger.recordOutput("Calibration/Summary", summary.toString());

        // Also log as individual entries for easy filtering
        for (int i = 0; i < successfulShots.size(); i++) {
            ShotConfiguration config = successfulShots.get(i);
            Logger.recordOutput(
                    "Calibration/Success/" + i + "/Distance", config.distance().in(Meters));
            Logger.recordOutput(
                    "Calibration/Success/" + i + "/Angle", config.hoodAngle().in(Degrees));
            Logger.recordOutput(
                    "Calibration/Success/" + i + "/RPM",
                    config.flywheelVelocity().in(RPM));
        }

        // Print to console as well
        System.out.println(summary.toString());
    }

    @Override
    public boolean isFinished() {
        return state == CalibrationState.COMPLETE;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        if (interrupted) {
            Logger.recordOutput("Calibration/Status", "INTERRUPTED");
        } else {
            logResults();
        }
    }

    /**
     * Get all successful shot configurations found during calibration.
     *
     * @return List of successful configurations
     */
    public List<ShotConfiguration> getSuccessfulShots() {
        return new ArrayList<>(successfulShots);
    }

    /**
     * Get all test results.
     *
     * @return List of all configurations tested
     */
    public List<ShotConfiguration> getAllResults() {
        return new ArrayList<>(results);
    }
}
