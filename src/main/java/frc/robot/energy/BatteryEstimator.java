// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.energy;

import java.util.Objects;

/**
 * Battery state estimator using a single RC Thevenin model with Peukert correction.
 *
 * <p>The model structure and MK Powered reference fit are adapted from Mechanical Advantage's 2026
 * {@code BatteryEstimator}. State of charge is determined from open-loop coulomb counting, and polarization voltage is
 * corrected from terminal-voltage measurements with a scalar Kalman gain.
 */
public final class BatteryEstimator {
    private final Parameters parameters;

    private double stateOfCharge;
    private double polarizationVoltage;
    private double polarizationVariance;

    public BatteryEstimator(Parameters parameters) {
        this.parameters = Objects.requireNonNull(parameters);
        reset(12.5, 0.0);
    }

    /**
     * Resets the estimator from a terminal-voltage and current observation.
     *
     * @param terminalVoltage terminal battery voltage
     * @param currentAmps nonnegative total battery current
     */
    public void reset(double terminalVoltage, double currentAmps) {
        requirePositiveFinite(terminalVoltage, "terminalVoltage");
        requireNonnegativeFinite(currentAmps, "currentAmps");

        double estimatedOpenCircuitVoltage = terminalVoltage
                + currentAmps * (parameters.seriesResistanceBase + parameters.polarizationResistanceBase);
        stateOfCharge = clamp(calculateStateOfCharge(estimatedOpenCircuitVoltage), 0.0, 1.0);
        polarizationVoltage = currentAmps * polarizationResistance(stateOfCharge);
        polarizationVariance = parameters.polarizationProcessVariance;
    }

    /**
     * Advances the estimator using an explicit sample period.
     *
     * @param totalCurrentAmps nonnegative total battery current
     * @param terminalVoltage measured terminal voltage
     * @param dtSeconds positive elapsed time since the previous update
     */
    public void update(double totalCurrentAmps, double terminalVoltage, double dtSeconds) {
        requireNonnegativeFinite(totalCurrentAmps, "totalCurrentAmps");
        requirePositiveFinite(terminalVoltage, "terminalVoltage");
        requirePositiveFinite(dtSeconds, "dtSeconds");

        double effectiveCurrentAmps = totalCurrentAmps;
        if (totalCurrentAmps > parameters.nominalCurrentAmps) {
            double peukertExponent =
                    parameters.peukertBase + parameters.peukertStateOfChargeScale * (1.0 - stateOfCharge);
            effectiveCurrentAmps = totalCurrentAmps
                    * Math.pow(totalCurrentAmps / parameters.nominalCurrentAmps, peukertExponent - 1.0);
        }
        stateOfCharge =
                clamp(stateOfCharge - effectiveCurrentAmps * dtSeconds / parameters.capacityAmpSeconds, 0.0, 1.0);

        double polarizationResistance = polarizationResistance(stateOfCharge);
        double timeConstantSeconds = polarizationResistance * parameters.polarizationCapacitanceFarads;
        double steadyStatePolarizationVoltage = totalCurrentAmps * polarizationResistance;
        double decay = Math.exp(-dtSeconds / timeConstantSeconds);
        polarizationVoltage =
                steadyStatePolarizationVoltage - (steadyStatePolarizationVoltage - polarizationVoltage) * decay;
        polarizationVariance =
                decay * decay * polarizationVariance + parameters.polarizationProcessVariance * dtSeconds;

        double seriesResistance = seriesResistance(stateOfCharge);
        double predictedVoltage =
                calculateOpenCircuitVoltage(stateOfCharge) - totalCurrentAmps * seriesResistance - polarizationVoltage;
        double measurementVariance = parameters.terminalVoltageMeasurementVariance
                + parameters.seriesResistanceUncertainty * totalCurrentAmps * totalCurrentAmps;
        double kalmanGain = polarizationVariance / (polarizationVariance + measurementVariance);
        polarizationVoltage += kalmanGain * (predictedVoltage - terminalVoltage);
        polarizationVariance *= 1.0 - kalmanGain;
    }

    /** Returns the maximum modeled current that keeps terminal voltage at or above the given floor. */
    public double maxCurrentAtVoltage(double minimumVoltage) {
        requireNonnegativeFinite(minimumVoltage, "minimumVoltage");
        return Math.max(
                0.0,
                (calculateOpenCircuitVoltage(stateOfCharge) - polarizationVoltage - minimumVoltage)
                        / seriesResistance(stateOfCharge));
    }

    public double stateOfCharge() {
        return stateOfCharge;
    }

    public double polarizationVoltage() {
        return polarizationVoltage;
    }

    public double polarizationVariance() {
        return polarizationVariance;
    }

    public double estimatedTerminalVoltage(double totalCurrentAmps) {
        requireNonnegativeFinite(totalCurrentAmps, "totalCurrentAmps");
        return calculateOpenCircuitVoltage(stateOfCharge)
                - totalCurrentAmps * seriesResistance(stateOfCharge)
                - polarizationVoltage;
    }

    private double seriesResistance(double stateOfCharge) {
        return parameters.seriesResistanceBase
                + parameters.seriesResistanceKneeGain
                        * Math.exp(parameters.seriesResistanceKneeRate
                                * (parameters.seriesResistanceKneeStateOfCharge - stateOfCharge));
    }

    private double polarizationResistance(double stateOfCharge) {
        double dischargedFraction = 1.0 - stateOfCharge;
        return parameters.polarizationResistanceBase
                * (1.0 + parameters.polarizationResistanceStateOfChargeScale * dischargedFraction * dischargedFraction);
    }

    private double calculateOpenCircuitVoltage(double stateOfCharge) {
        double clampedStateOfCharge = clamp(stateOfCharge, 0.0, 1.0);
        for (int i = 0; i < parameters.openCircuitVoltageStateOfChargeKnots.length - 1; i++) {
            if (clampedStateOfCharge <= parameters.openCircuitVoltageStateOfChargeKnots[i + 1]) {
                double interpolation = (clampedStateOfCharge - parameters.openCircuitVoltageStateOfChargeKnots[i])
                        / (parameters.openCircuitVoltageStateOfChargeKnots[i + 1]
                                - parameters.openCircuitVoltageStateOfChargeKnots[i]);
                return parameters.openCircuitVoltageVolts[i]
                        + interpolation
                                * (parameters.openCircuitVoltageVolts[i + 1] - parameters.openCircuitVoltageVolts[i]);
            }
        }
        return parameters.openCircuitVoltageVolts[parameters.openCircuitVoltageVolts.length - 1];
    }

    private double calculateStateOfCharge(double openCircuitVoltage) {
        double clampedOpenCircuitVoltage = clamp(
                openCircuitVoltage,
                parameters.openCircuitVoltageVolts[0],
                parameters.openCircuitVoltageVolts[parameters.openCircuitVoltageVolts.length - 1]);
        for (int i = 0; i < parameters.openCircuitVoltageVolts.length - 1; i++) {
            if (clampedOpenCircuitVoltage <= parameters.openCircuitVoltageVolts[i + 1]) {
                double interpolation = (clampedOpenCircuitVoltage - parameters.openCircuitVoltageVolts[i])
                        / (parameters.openCircuitVoltageVolts[i + 1] - parameters.openCircuitVoltageVolts[i]);
                return parameters.openCircuitVoltageStateOfChargeKnots[i]
                        + interpolation
                                * (parameters.openCircuitVoltageStateOfChargeKnots[i + 1]
                                        - parameters.openCircuitVoltageStateOfChargeKnots[i]);
            }
        }
        return parameters
                .openCircuitVoltageStateOfChargeKnots[parameters.openCircuitVoltageStateOfChargeKnots.length - 1];
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

    /** Immutable coefficients for a battery-model fit. */
    public static final class Parameters {
        private final double capacityAmpSeconds;
        private final double[] openCircuitVoltageStateOfChargeKnots;
        private final double[] openCircuitVoltageVolts;
        private final double seriesResistanceBase;
        private final double seriesResistanceKneeGain;
        private final double seriesResistanceKneeRate;
        private final double seriesResistanceKneeStateOfCharge;
        private final double polarizationResistanceBase;
        private final double polarizationResistanceStateOfChargeScale;
        private final double polarizationCapacitanceFarads;
        private final double peukertBase;
        private final double peukertStateOfChargeScale;
        private final double nominalCurrentAmps;
        private final double polarizationProcessVariance;
        private final double terminalVoltageMeasurementVariance;
        private final double seriesResistanceUncertainty;

        private Parameters(
                double capacityAmpHours,
                double[] openCircuitVoltageStateOfChargeKnots,
                double[] openCircuitVoltageVolts,
                double seriesResistanceBase,
                double seriesResistanceKneeGain,
                double seriesResistanceKneeRate,
                double seriesResistanceKneeStateOfCharge,
                double polarizationResistanceBase,
                double polarizationResistanceStateOfChargeScale,
                double polarizationCapacitanceFarads,
                double peukertBase,
                double peukertStateOfChargeScale,
                double nominalCurrentAmps,
                double polarizationProcessVariance,
                double terminalVoltageMeasurementVariance,
                double seriesResistanceUncertainty) {
            requirePositiveFinite(capacityAmpHours, "capacityAmpHours");
            validateCurve(openCircuitVoltageStateOfChargeKnots, openCircuitVoltageVolts);
            requirePositiveFinite(seriesResistanceBase, "seriesResistanceBase");
            requireNonnegativeFinite(seriesResistanceKneeGain, "seriesResistanceKneeGain");
            requireNonnegativeFinite(seriesResistanceKneeRate, "seriesResistanceKneeRate");
            requireNonnegativeFinite(seriesResistanceKneeStateOfCharge, "seriesResistanceKneeStateOfCharge");
            if (seriesResistanceKneeStateOfCharge > 1.0) {
                throw new IllegalArgumentException("seriesResistanceKneeStateOfCharge must be at most 1.0");
            }
            requirePositiveFinite(polarizationResistanceBase, "polarizationResistanceBase");
            requireNonnegativeFinite(
                    polarizationResistanceStateOfChargeScale, "polarizationResistanceStateOfChargeScale");
            requirePositiveFinite(polarizationCapacitanceFarads, "polarizationCapacitanceFarads");
            requirePositiveFinite(peukertBase, "peukertBase");
            requireNonnegativeFinite(peukertStateOfChargeScale, "peukertStateOfChargeScale");
            requirePositiveFinite(nominalCurrentAmps, "nominalCurrentAmps");
            requireNonnegativeFinite(polarizationProcessVariance, "polarizationProcessVariance");
            requirePositiveFinite(terminalVoltageMeasurementVariance, "terminalVoltageMeasurementVariance");
            requireNonnegativeFinite(seriesResistanceUncertainty, "seriesResistanceUncertainty");

            this.capacityAmpSeconds = capacityAmpHours * 3600.0;
            this.openCircuitVoltageStateOfChargeKnots = openCircuitVoltageStateOfChargeKnots.clone();
            this.openCircuitVoltageVolts = openCircuitVoltageVolts.clone();
            this.seriesResistanceBase = seriesResistanceBase;
            this.seriesResistanceKneeGain = seriesResistanceKneeGain;
            this.seriesResistanceKneeRate = seriesResistanceKneeRate;
            this.seriesResistanceKneeStateOfCharge = seriesResistanceKneeStateOfCharge;
            this.polarizationResistanceBase = polarizationResistanceBase;
            this.polarizationResistanceStateOfChargeScale = polarizationResistanceStateOfChargeScale;
            this.polarizationCapacitanceFarads = polarizationCapacitanceFarads;
            this.peukertBase = peukertBase;
            this.peukertStateOfChargeScale = peukertStateOfChargeScale;
            this.nominalCurrentAmps = nominalCurrentAmps;
            this.polarizationProcessVariance = polarizationProcessVariance;
            this.terminalVoltageMeasurementVariance = terminalVoltageMeasurementVariance;
            this.seriesResistanceUncertainty = seriesResistanceUncertainty;
        }

        /**
         * Returns the 6328 MK Powered reference fit.
         *
         * <p>These values are a starting reference, not a characterization of this robot's batteries.
         */
        public static Parameters mechanicalAdvantageMkPoweredReference() {
            double capacityAmpHours = 19.75;
            return new Parameters(
                    capacityAmpHours,
                    new double[] {0.0, 0.2, 0.4, 0.6, 0.8, 1.0},
                    new double[] {12.112, 12.113, 12.311, 12.401, 12.535, 13.196},
                    0.01181,
                    0.00335,
                    1.36726,
                    0.4,
                    0.00274,
                    2.10529,
                    400.0,
                    1.07,
                    0.1,
                    capacityAmpHours / 20.0,
                    Math.pow(0.02, 2.0),
                    Math.pow(0.1, 2.0),
                    Math.pow(0.0015, 2.0));
        }

        public double[] openCircuitVoltageStateOfChargeKnots() {
            return openCircuitVoltageStateOfChargeKnots.clone();
        }

        public double[] openCircuitVoltageVolts() {
            return openCircuitVoltageVolts.clone();
        }

        private static void validateCurve(double[] stateOfChargeKnots, double[] voltages) {
            Objects.requireNonNull(stateOfChargeKnots);
            Objects.requireNonNull(voltages);
            if (stateOfChargeKnots.length < 2 || stateOfChargeKnots.length != voltages.length) {
                throw new IllegalArgumentException(
                        "open-circuit-voltage curves must have equal lengths of at least two");
            }
            for (int i = 0; i < stateOfChargeKnots.length; i++) {
                if (!Double.isFinite(stateOfChargeKnots[i])
                        || stateOfChargeKnots[i] < 0.0
                        || stateOfChargeKnots[i] > 1.0) {
                    throw new IllegalArgumentException("state-of-charge knots must be finite and within [0, 1]");
                }
                requirePositiveFinite(voltages[i], "openCircuitVoltage");
                if (i > 0 && (stateOfChargeKnots[i] <= stateOfChargeKnots[i - 1] || voltages[i] <= voltages[i - 1])) {
                    throw new IllegalArgumentException(
                            "state-of-charge knots and voltages must be strictly increasing");
                }
            }
        }
    }
}
