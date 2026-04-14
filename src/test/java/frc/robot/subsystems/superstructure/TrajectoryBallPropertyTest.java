package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.shooter.ShooterConstants;
import net.jqwik.api.*;
import net.jqwik.api.constraints.DoubleRange;
import net.jqwik.api.lifecycle.BeforeContainer;

class TrajectoryBallPropertyTest {

    @BeforeContainer
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    // ===== Property: Launch angle is always the fixed hood angle =====

    @Property
    void launchAngleIsAlwaysFixed(@ForAll @DoubleRange(min = 0.1, max = 20.0) double distanceMeters) {
        var result = TrajectoryBall.calculateStationaryMap(Meters.of(distanceMeters));
        assertEquals(
                ShooterConstants.kFixedHoodAngle.in(Radians),
                result.launchAngle().in(Radians),
                1e-9,
                "Launch angle must equal fixed hood angle at distance=" + distanceMeters);
    }

    // ===== Property: Launch speed is always non-negative =====

    @Property
    void launchSpeedIsNonNegative(@ForAll @DoubleRange(min = 0.0, max = 20.0) double distanceMeters) {
        var result = TrajectoryBall.calculateStationaryMap(Meters.of(distanceMeters));
        assertTrue(
                result.launchSpeed().in(MetersPerSecond) >= 0,
                "Launch speed must be non-negative at distance=" + distanceMeters);
    }

    // ===== Property: Time of flight is positive for non-zero distance =====

    @Property
    void timeOfFlightIsPositive(@ForAll @DoubleRange(min = 0.1, max = 20.0) double distanceMeters) {
        var result = TrajectoryBall.calculateStationaryMap(Meters.of(distanceMeters));
        assertTrue(result.totalTime() > 0, "Time of flight must be positive at distance=" + distanceMeters);
    }

    // ===== Property: Farther distance → higher launch speed (monotonic) =====

    @Property
    void launchSpeedIncreasesWithDistance(
            @ForAll @DoubleRange(min = 0.1, max = 19.0) double nearMeters,
            @ForAll @DoubleRange(min = 0.1, max = 19.0) double farMeters) {
        Assume.that(farMeters > nearMeters + 0.5); // Ensure meaningful gap
        var near = TrajectoryBall.calculateStationaryMap(Meters.of(nearMeters));
        var far = TrajectoryBall.calculateStationaryMap(Meters.of(farMeters));
        assertTrue(
                far.launchSpeed().in(MetersPerSecond) >= near.launchSpeed().in(MetersPerSecond),
                "Launch speed at " + farMeters + "m should be >= speed at " + nearMeters + "m");
    }

    // ===== Property: Flywheel RPM is within physical bounds =====

    @Property
    void flywheelRpmIsReasonable(@ForAll @DoubleRange(min = 0.0, max = 15.0) double distanceMeters) {
        double rpm = TrajectoryBall.getFlywheelVelocityForDistance(Meters.of(distanceMeters))
                .in(RPM);
        assertTrue(rpm >= 0, "RPM must be non-negative");
        assertTrue(rpm <= 8000, "RPM must be physically reasonable (<=8000)");
    }
}
