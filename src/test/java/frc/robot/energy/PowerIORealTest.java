package frc.robot.energy;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class PowerIORealTest {
    @Test
    void requiresPlausiblePowerDistributionVoltageAndCurrent() {
        assertTrue(PowerIOReal.isValidSample(120.0, 12.0));
        assertTrue(PowerIOReal.isValidSample(0.0, 12.0));

        assertFalse(PowerIOReal.isValidSample(Double.NaN, 12.0));
        assertFalse(PowerIOReal.isValidSample(-1.0, 12.0));
        assertFalse(PowerIOReal.isValidSample(120.0, Double.NaN));
        assertFalse(PowerIOReal.isValidSample(120.0, 0.0));
    }
}
