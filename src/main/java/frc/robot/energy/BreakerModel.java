// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.energy;

/**
 * Thermal model of the 120 A main breaker using Miner's-rule damage accumulation.
 *
 * <p>The model and breaker-curve points are adapted from Mechanical Advantage's 2026 {@code BreakerModel}. Niceness
 * scales down the modeled trip threshold so throttling starts before the actual trip point.
 */
public final class BreakerModel {
    public static final double RATED_CURRENT_AMPS = 120.0;

    private static final double COOLING_TIME_CONSTANT_SECONDS = 60.0;
    private static final double[] NORMALIZED_CURRENT_POINTS = {1.0, 1.35, 2.0, 2.25, 2.5, 3.0, 4.0, 5.0};
    private static final double[] TRIP_TIME_SECONDS = {1.0e6, 30.0 * 60.0, 70.0, 38.0, 25.0, 15.0, 10.0, 7.0};

    private final double tripThreshold;
    private double damage;

    public BreakerModel(double niceness) {
        if (!Double.isFinite(niceness) || niceness < 0.0 || niceness > 1.0) {
            throw new IllegalArgumentException("niceness must be finite and within [0, 1]");
        }
        tripThreshold = 1.0 - niceness;
    }

    /**
     * Advances accumulated breaker damage using an explicit sample period.
     *
     * @param totalCurrentAmps nonnegative total breaker current
     * @param dtSeconds positive elapsed time
     */
    public void update(double totalCurrentAmps, double dtSeconds) {
        requireNonnegativeFinite(totalCurrentAmps, "totalCurrentAmps");
        requirePositiveFinite(dtSeconds, "dtSeconds");

        double normalizedCurrent = totalCurrentAmps / RATED_CURRENT_AMPS;
        if (normalizedCurrent > 1.0) {
            damage += dtSeconds / tripTimeSeconds(normalizedCurrent);
        } else {
            damage *= Math.exp(-dtSeconds / COOLING_TIME_CONSTANT_SECONDS);
        }
        damage = clamp(damage, 0.0, 1.0);
    }

    /** Returns the current that consumes the remaining allowed damage over the requested horizon. */
    public double maxCurrentFor(double horizonSeconds) {
        requirePositiveFinite(horizonSeconds, "horizonSeconds");

        double remainingDamage = tripThreshold - damage;
        if (remainingDamage <= 0.0) {
            return 0.0;
        }

        double requiredTripTimeSeconds = horizonSeconds / remainingDamage;
        if (requiredTripTimeSeconds >= TRIP_TIME_SECONDS[0]) {
            return Double.MAX_VALUE;
        }
        if (requiredTripTimeSeconds <= TRIP_TIME_SECONDS[TRIP_TIME_SECONDS.length - 1]) {
            return NORMALIZED_CURRENT_POINTS[NORMALIZED_CURRENT_POINTS.length - 1] * RATED_CURRENT_AMPS;
        }

        return inverseTripTime(requiredTripTimeSeconds) * RATED_CURRENT_AMPS;
    }

    public double damage() {
        return damage;
    }

    /** Returns logarithmically interpolated breaker trip time for normalized current. */
    public static double tripTimeSeconds(double normalizedCurrent) {
        requireNonnegativeFinite(normalizedCurrent, "normalizedCurrent");
        double clampedCurrent = clamp(
                normalizedCurrent,
                NORMALIZED_CURRENT_POINTS[0],
                NORMALIZED_CURRENT_POINTS[NORMALIZED_CURRENT_POINTS.length - 1]);
        return interpolateLog(clampedCurrent, NORMALIZED_CURRENT_POINTS, TRIP_TIME_SECONDS);
    }

    private static double inverseTripTime(double tripTimeSeconds) {
        return interpolateLog(tripTimeSeconds, TRIP_TIME_SECONDS, NORMALIZED_CURRENT_POINTS);
    }

    private static double interpolateLog(double query, double[] inputs, double[] outputs) {
        for (int i = 0; i < inputs.length - 1; i++) {
            double lower = Math.min(inputs[i], inputs[i + 1]);
            double upper = Math.max(inputs[i], inputs[i + 1]);
            if (query >= lower && query <= upper) {
                double interpolation =
                        (Math.log(query) - Math.log(inputs[i])) / (Math.log(inputs[i + 1]) - Math.log(inputs[i]));
                return Math.exp(
                        Math.log(outputs[i]) + interpolation * (Math.log(outputs[i + 1]) - Math.log(outputs[i])));
            }
        }
        throw new IllegalArgumentException("query is outside interpolation curve");
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(value, maximum));
    }

    private static void requireNonnegativeFinite(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and nonnegative");
        }
    }

    private static void requirePositiveFinite(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and positive");
        }
    }
}
