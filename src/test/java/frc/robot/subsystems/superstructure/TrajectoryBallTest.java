package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class TrajectoryBallTest {

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    // ===== calculateStationaryMap Tests =====

    @Test
    void stationaryMapReturnsFixedHoodAngle() {
        var result = TrajectoryBall.calculateStationaryMap(Meters.of(3.0));
        assertEquals(
                ShooterConstants.kFixedHoodAngle.in(Radians),
                result.launchAngle().in(Radians),
                1e-9,
                "Launch angle should always equal the fixed hood angle");
    }

    @Test
    void stationaryMapReturnsPositiveLaunchSpeed() {
        var result = TrajectoryBall.calculateStationaryMap(Meters.of(3.0));
        assertTrue(result.launchSpeed().in(MetersPerSecond) > 0, "Launch speed should be positive for valid distance");
    }

    @Test
    void stationaryMapReturnsPositiveTimeOfFlight() {
        var result = TrajectoryBall.calculateStationaryMap(Meters.of(3.0));
        assertTrue(result.totalTime() > 0, "Time of flight should be positive");
    }

    @Test
    void stationaryMapZeroDistanceGivesMinimalSpeed() {
        var result = TrajectoryBall.calculateStationaryMap(Meters.of(0.0));
        // At 0m distance, the interpolation map gives 1500 RPM (minimum)
        // The resulting launch speed should be small but non-negative
        assertTrue(
                result.launchSpeed().in(MetersPerSecond) >= 0, "Launch speed at zero distance should be non-negative");
    }

    @Test
    void stationaryMapLargerDistanceGivesFasterSpeed() {
        var near = TrajectoryBall.calculateStationaryMap(Meters.of(1.0));
        var far = TrajectoryBall.calculateStationaryMap(Meters.of(5.0));
        assertTrue(
                far.launchSpeed().in(MetersPerSecond) > near.launchSpeed().in(MetersPerSecond),
                "Farther targets should require faster launch speeds");
    }

    // ===== getFlywheelVelocityForDistance Tests =====

    @Test
    void flywheelVelocityInterpolatesCorrectly() {
        // The map has entries at 0m→1500 RPM, 1m→2400 RPM, 3.18m→3600 RPM, 6m→4000 RPM
        // (with offsetM=0.5969 added to each key)
        // At a known point, the velocity should match
        var velocity = TrajectoryBall.getFlywheelVelocityForDistance(Meters.of(1.0 + ShooterConstants.offsetM));
        assertEquals(2400.0, velocity.in(RPM), 1.0, "Should match map entry at 1m + offset");
    }

    @Test
    void flywheelVelocityIncreaseWithDistance() {
        var near = TrajectoryBall.getFlywheelVelocityForDistance(Meters.of(1.0));
        var far = TrajectoryBall.getFlywheelVelocityForDistance(Meters.of(5.0));
        assertTrue(far.in(RPM) > near.in(RPM), "Farther distance should require higher flywheel RPM");
    }

    @Test
    void flywheelVelocityIsWithinReasonableRange() {
        Distance[] distances = {Meters.of(0.5), Meters.of(2.0), Meters.of(4.0), Meters.of(6.0)};
        for (Distance d : distances) {
            double rpm = TrajectoryBall.getFlywheelVelocityForDistance(d).in(RPM);
            assertTrue(rpm >= 0, "RPM should be non-negative at distance=" + d.in(Meters) + "m");
            assertTrue(rpm <= 10000, "RPM should be reasonable at distance=" + d.in(Meters) + "m");
        }
    }
}
