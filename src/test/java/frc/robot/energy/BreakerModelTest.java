package frc.robot.energy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class BreakerModelTest {
    private static final double EPSILON = 1.0e-12;

    @Test
    void matchesMainBreakerDatasheetTripPoints() {
        assertEquals(1800.0, BreakerModel.tripTimeSeconds(1.35), EPSILON);
        assertEquals(70.0, BreakerModel.tripTimeSeconds(2.0), EPSILON);
        assertEquals(7.0, BreakerModel.tripTimeSeconds(5.0), EPSILON);
    }

    @Test
    void accumulatesDamageAndCoolsExponentially() {
        BreakerModel breaker = new BreakerModel(0.05);

        breaker.update(240.0, 7.0);
        assertEquals(0.1, breaker.damage(), EPSILON);

        breaker.update(120.0, 60.0);
        assertEquals(0.0367879441171442, breaker.damage(), EPSILON);
    }

    @Test
    void calculatesCurrentFromRemainingDamageAndRequestedHorizon() {
        BreakerModel breaker = new BreakerModel(0.05);
        breaker.update(240.0, 63.0);

        assertEquals(0.9, breaker.damage(), EPSILON);
        assertEquals(247.239898685823, breaker.maxCurrentFor(3.0), 1.0e-9);
    }

    @Test
    void fullyDamagedBreakerHasNoRemainingCurrentBudget() {
        BreakerModel breaker = new BreakerModel(0.05);

        breaker.update(600.0, 7.0);

        assertEquals(1.0, breaker.damage(), EPSILON);
        assertEquals(0.0, breaker.maxCurrentFor(3.0), EPSILON);
    }

    @Test
    void rejectsInvalidInputs() {
        assertThrows(IllegalArgumentException.class, () -> new BreakerModel(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> new BreakerModel(-0.01));
        assertThrows(IllegalArgumentException.class, () -> new BreakerModel(1.01));

        BreakerModel breaker = new BreakerModel(0.05);
        assertThrows(IllegalArgumentException.class, () -> breaker.update(-1.0, 0.02));
        assertThrows(IllegalArgumentException.class, () -> breaker.update(1.0, 0.0));
        assertThrows(IllegalArgumentException.class, () -> breaker.maxCurrentFor(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> breaker.maxCurrentFor(0.0));
        assertThrows(IllegalArgumentException.class, () -> BreakerModel.tripTimeSeconds(-1.0));
    }
}
