package frc.robot.energy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class BatteryEstimatorTest {
    private static final double EPSILON = 1.0e-12;

    @Test
    void resetsFromTheMechanicalAdvantageMkPoweredReferenceCurve() {
        BatteryEstimator estimator =
                new BatteryEstimator(BatteryEstimator.Parameters.mechanicalAdvantageMkPoweredReference());

        estimator.reset(12.401, 0.0);

        assertEquals(0.6, estimator.stateOfCharge(), EPSILON);
        assertEquals(0.0, estimator.polarizationVoltage(), EPSILON);
        assertEquals(376.153271262299, estimator.maxCurrentAtVoltage(7.0), 1.0e-9);
    }

    @Test
    void returnsZeroWhenTheVoltageFloorIsAboveAvailableOpenCircuitVoltage() {
        BatteryEstimator estimator =
                new BatteryEstimator(BatteryEstimator.Parameters.mechanicalAdvantageMkPoweredReference());
        estimator.reset(12.401, 0.0);

        assertEquals(0.0, estimator.maxCurrentAtVoltage(12.5), EPSILON);
    }

    @Test
    void noLoadUpdateIsStableAndDischargeLowersStateOfCharge() {
        BatteryEstimator estimator =
                new BatteryEstimator(BatteryEstimator.Parameters.mechanicalAdvantageMkPoweredReference());
        estimator.reset(12.401, 0.0);

        estimator.update(0.0, 12.401, 0.02);
        double noLoadStateOfCharge = estimator.stateOfCharge();
        estimator.update(200.0, 9.0, 1.0);

        assertEquals(0.6, noLoadStateOfCharge, EPSILON);
        assertTrue(estimator.stateOfCharge() < noLoadStateOfCharge);
    }

    @Test
    void stateOfChargeIsClampedAfterLongDischarge() {
        BatteryEstimator estimator =
                new BatteryEstimator(BatteryEstimator.Parameters.mechanicalAdvantageMkPoweredReference());
        estimator.reset(12.112, 0.0);

        estimator.update(600.0, 7.0, 10_000.0);

        assertEquals(0.0, estimator.stateOfCharge(), EPSILON);
    }

    @Test
    void referenceParameterArraysAreDefensiveCopies() {
        BatteryEstimator.Parameters parameters = BatteryEstimator.Parameters.mechanicalAdvantageMkPoweredReference();
        double[] knots = parameters.openCircuitVoltageStateOfChargeKnots();
        double original = knots[0];

        knots[0] = 0.5;

        assertEquals(original, parameters.openCircuitVoltageStateOfChargeKnots()[0], EPSILON);
    }

    @Test
    void rejectsInvalidMeasurementsAndTimeSteps() {
        BatteryEstimator estimator =
                new BatteryEstimator(BatteryEstimator.Parameters.mechanicalAdvantageMkPoweredReference());

        assertThrows(IllegalArgumentException.class, () -> estimator.reset(Double.NaN, 0.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.reset(12.0, -1.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.update(-1.0, 12.0, 0.02));
        assertThrows(IllegalArgumentException.class, () -> estimator.update(1.0, Double.NaN, 0.02));
        assertThrows(IllegalArgumentException.class, () -> estimator.update(1.0, 12.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> estimator.maxCurrentAtVoltage(Double.NaN));
    }
}
