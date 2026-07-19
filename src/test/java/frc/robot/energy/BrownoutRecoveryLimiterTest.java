package frc.robot.energy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class BrownoutRecoveryLimiterTest {
    private static final double EPSILON = 1e-9;

    @Test
    void decreasesImmediatelyAndRateLimitsRecovery() {
        BrownoutRecoveryLimiter limiter = new BrownoutRecoveryLimiter(50.0, 2.0);
        limiter.reset(120.0);

        assertEquals(70.0, limiter.update(70.0, true, 0.02), EPSILON);
        assertTrue(limiter.isRecovering());

        assertEquals(71.0, limiter.update(120.0, false, 0.02), EPSILON);
        assertEquals(72.0, limiter.update(120.0, false, 0.02), EPSILON);
        assertTrue(limiter.isRecovering());
    }

    @Test
    void staysRateLimitedAfterTheMinimumWindowUntilItCatchesTheRawBudget() {
        BrownoutRecoveryLimiter limiter = new BrownoutRecoveryLimiter(10.0, 0.05);
        limiter.reset(20.0);

        limiter.update(10.0, true, 0.01);
        assertEquals(10.2, limiter.update(30.0, false, 0.02), EPSILON);
        assertEquals(10.4, limiter.update(30.0, false, 0.02), EPSILON);
        assertEquals(10.6, limiter.update(30.0, false, 0.02), EPSILON);
        assertTrue(limiter.isRecovering());

        assertEquals(10.6, limiter.update(10.6, false, 0.02), EPSILON);
        assertTrue(limiter.isRecovering());
        for (int i = 0; i < 100; i++) {
            limiter.update(30.0, false, 0.02);
        }
        assertEquals(30.0, limiter.getFilteredBudgetAmps(), EPSILON);
        assertFalse(limiter.isRecovering());
    }

    @Test
    void aRawDipDoesNotAllowTheNextReboundToBypassRecoveryRateLimiting() {
        BrownoutRecoveryLimiter limiter = new BrownoutRecoveryLimiter(10.0, 0.05);
        limiter.reset(30.0);

        limiter.update(30.0, true, 0.01);
        assertEquals(30.2, limiter.update(100.0, false, 0.02), EPSILON);
        assertEquals(30.4, limiter.update(100.0, false, 0.02), EPSILON);
        assertEquals(30.6, limiter.update(100.0, false, 0.02), EPSILON);

        assertEquals(20.0, limiter.update(20.0, false, 0.02), EPSILON);
        assertTrue(limiter.isRecovering());
        assertEquals(20.2, limiter.update(100.0, false, 0.02), EPSILON);
        assertTrue(limiter.isRecovering());
    }

    @Test
    void resetClearsRecoveryState() {
        BrownoutRecoveryLimiter limiter = new BrownoutRecoveryLimiter(50.0, 2.0);
        limiter.reset(100.0);
        limiter.update(50.0, true, 0.02);

        limiter.reset(80.0);

        assertEquals(80.0, limiter.getFilteredBudgetAmps(), EPSILON);
        assertFalse(limiter.isRecovering());
        assertEquals(100.0, limiter.update(100.0, false, 0.02), EPSILON);
    }

    @Test
    void rejectsInvalidConfigurationAndSamples() {
        assertThrows(IllegalArgumentException.class, () -> new BrownoutRecoveryLimiter(0.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new BrownoutRecoveryLimiter(1.0, -1.0));

        BrownoutRecoveryLimiter limiter = new BrownoutRecoveryLimiter(50.0, 2.0);
        assertThrows(IllegalArgumentException.class, () -> limiter.reset(Double.NaN));
        assertThrows(IllegalStateException.class, () -> limiter.update(10.0, false, 0.02));

        limiter.reset(10.0);
        assertThrows(IllegalArgumentException.class, () -> limiter.update(-1.0, false, 0.02));
        assertThrows(IllegalArgumentException.class, () -> limiter.update(10.0, false, 0.0));
        assertThrows(IllegalArgumentException.class, () -> limiter.update(10.0, false, Double.NaN));
    }
}
