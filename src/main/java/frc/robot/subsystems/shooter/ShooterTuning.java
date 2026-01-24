package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTuning {
    // Map from distance (meters) to angular velocity (rad/s)
    private static final InterpolatingDoubleTreeMap distanceToAngularVelocity = new InterpolatingDoubleTreeMap();

    // Keys for tuning
    private static final double[] distances = {2.0, 3.0, 4.0, 5.0, 6.0};
    // Default angular velocities in rad/s for each distance
    private static final double[] defaultAngularVelocities = {50.0, 55.0, 60.0, 65.0, 70.0};

    static {
        for (int i = 0; i < distances.length; i++) {
            distanceToAngularVelocity.put(distances[i], defaultAngularVelocities[i]);
            // Initialize dashboard with default values
            SmartDashboard.setDefaultNumber(
                    "ShooterTuning/AngularVelocity " + distances[i] + "m (rad_s)", defaultAngularVelocities[i]);
        }
    }

    /**
     * Updates the map with the latest tunable values from the dashboard. Call this method periodically or before
     * querying the map.
     */
    private static void updateMap() {
        for (double dist : distances) {
            double val = SmartDashboard.getNumber(
                    "ShooterTuning/AngularVelocity " + dist + "m (rad_s)", defaultAngularVelocities[getIndex(dist)]);
            distanceToAngularVelocity.put(dist, val);
        }
    }

    private static int getIndex(double dist) {
        for (int i = 0; i < distances.length; i++) {
            if (distances[i] == dist) return i;
        }
        return 0;
    }

    /**
     * Calculates the shooter angular velocity for a given distance.
     *
     * @param distanceMeters The distance to the target in meters.
     * @return The target angular velocity.
     */
    public static AngularVelocity getAngularVelocity(double distanceMeters) {
        updateMap();
        return RadiansPerSecond.of(distanceToAngularVelocity.get(distanceMeters));
    }
}
