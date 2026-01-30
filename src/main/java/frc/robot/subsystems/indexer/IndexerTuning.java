package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerTuning {
    // Map from distance (meters) to angular velocity (rad/s)
    private final InterpolatingDoubleTreeMap distanceToAngularVelocity = new InterpolatingDoubleTreeMap();

    // Keys for tuning - distance points
    private final Distance[] distances = {Meters.of(2.0), Meters.of(3.0), Meters.of(4.0), Meters.of(5.0), Meters.of(6.0)
    };
    // Default angular velocities for each distance
    private final AngularVelocity[] defaultAngularVelocities = {
        RadiansPerSecond.of(1),
        RadiansPerSecond.of(55.0),
        RadiansPerSecond.of(60.0),
        RadiansPerSecond.of(65.0),
        RadiansPerSecond.of(70.0)
    };

    public IndexerTuning() {
        // Initialize distance-to-velocity tuning
        for (int i = 0; i < distances.length; i++) {
            distanceToAngularVelocity.put(distances[i].in(Meters), defaultAngularVelocities[i].in(RadiansPerSecond));
            SmartDashboard.setDefaultNumber(
                    "IndexerTuning/AngularVelocity " + distances[i].in(Meters) + "m (rad_s)",
                    defaultAngularVelocities[i].in(RadiansPerSecond));
        }

        // Initialize PID/FF gains for real robot (defaults from IndexerConstants)
        SmartDashboard.setDefaultNumber("IndexerTuning/kP", IndexerConstants.kP);
        SmartDashboard.setDefaultNumber("IndexerTuning/kI", IndexerConstants.kI);
        SmartDashboard.setDefaultNumber("IndexerTuning/kD", IndexerConstants.kD);
        SmartDashboard.setDefaultNumber("IndexerTuning/kS", IndexerConstants.kS);
        SmartDashboard.setDefaultNumber("IndexerTuning/kV", IndexerConstants.kV);

        // Initialize simulation PID gains (defaults from IndexerConstants)
        SmartDashboard.setDefaultNumber("IndexerTuning/Sim_kP", IndexerConstants.kSimP);
        SmartDashboard.setDefaultNumber("IndexerTuning/Sim_kI", IndexerConstants.kSimI);
        SmartDashboard.setDefaultNumber("IndexerTuning/Sim_kD", IndexerConstants.kSimD);

        // Initialize current limit (default from IndexerConstants)
        SmartDashboard.setDefaultNumber(
                "IndexerTuning/StatorCurrentLimit (A)", IndexerConstants.kStatorCurrentLimit.in(Amps));
    }

    /** Updates the distance-to-velocity map with the latest tunable values from the dashboard. */
    private void updateMap() {
        for (int i = 0; i < distances.length; i++) {
            double val = SmartDashboard.getNumber(
                    "IndexerTuning/AngularVelocity " + distances[i].in(Meters) + "m (rad_s)",
                    defaultAngularVelocities[i].in(RadiansPerSecond));
            distanceToAngularVelocity.put(distances[i].in(Meters), val);
        }
    }

    /**
     * Calculates the Indexer angular velocity for a given distance.
     *
     * @param distance The distance to the target.
     * @return The target angular velocity.
     */
    public AngularVelocity getAngularVelocity(Distance distance) {
        updateMap();
        return RadiansPerSecond.of(distanceToAngularVelocity.get(distance.in(Meters)));
    }

    /**
     * Gets the current Slot0Configs with tunable PID/FF values from SmartDashboard.
     *
     * @return Slot0Configs with current tuned values.
     */
    public Slot0Configs getSlot0Configs() {
        return new Slot0Configs()
                .withKP(SmartDashboard.getNumber("IndexerTuning/kP", IndexerConstants.kP))
                .withKI(SmartDashboard.getNumber("IndexerTuning/kI", IndexerConstants.kI))
                .withKD(SmartDashboard.getNumber("IndexerTuning/kD", IndexerConstants.kD))
                .withKS(SmartDashboard.getNumber("IndexerTuning/kS", IndexerConstants.kS))
                .withKV(SmartDashboard.getNumber("IndexerTuning/kV", IndexerConstants.kV));
    }

    /** Gets the tunable kP value for simulation PID controller. */
    public double getSimKP() {
        return SmartDashboard.getNumber("IndexerTuning/Sim_kP", IndexerConstants.kSimP);
    }

    /** Gets the tunable kI value for simulation PID controller. */
    public double getSimKI() {
        return SmartDashboard.getNumber("IndexerTuning/Sim_kI", IndexerConstants.kSimI);
    }

    /** Gets the tunable kD value for simulation PID controller. */
    public double getSimKD() {
        return SmartDashboard.getNumber("IndexerTuning/Sim_kD", IndexerConstants.kSimD);
    }

    /** Gets the tunable stator current limit. */
    public Current getStatorCurrentLimit() {
        return Amps.of(SmartDashboard.getNumber(
                "IndexerTuning/StatorCurrentLimit (A)", IndexerConstants.kStatorCurrentLimit.in(Amps)));
    }

    /**
     * Gets the current Slot0Configs with tunable Simulation PID values from SmartDashboard. We pack Sim PIDs into
     * Slot0Configs for interface consistency.
     *
     * @return Slot0Configs with current tuned simulation values.
     */
    public Slot0Configs getSimSlot0Configs() {
        return new Slot0Configs()
                .withKP(getSimKP())
                .withKI(getSimKI())
                .withKD(getSimKD())
                .withKS(0.0)
                .withKV(0.0);
    }
}
