package frc.robot.energy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class BudgetMathTest {
    private static final double EPSILON = 1.0e-12;

    @Test
    void subtractsUncontrolledCurrentFromTheSafeRobotBudget() {
        BudgetMath.Result result = BudgetMath.derive(150.0, 180.0, 100.0);

        assertEquals(80.0, result.uncontrolledCurrentAmps(), EPSILON);
        assertEquals(70.0, result.controlledPoolAmps(), EPSILON);
        assertEquals(0.0, result.measurementMismatchAmps(), EPSILON);
    }

    @Test
    void clampsNegativeResidualAndReportsMeasurementMismatch() {
        BudgetMath.Result result = BudgetMath.derive(150.0, 50.0, 60.0);

        assertEquals(0.0, result.uncontrolledCurrentAmps(), EPSILON);
        assertEquals(150.0, result.controlledPoolAmps(), EPSILON);
        assertEquals(10.0, result.measurementMismatchAmps(), EPSILON);
    }

    @Test
    void clampsControlledPoolAtZero() {
        BudgetMath.Result result = BudgetMath.derive(50.0, 180.0, 100.0);

        assertEquals(80.0, result.uncontrolledCurrentAmps(), EPSILON);
        assertEquals(0.0, result.controlledPoolAmps(), EPSILON);
    }

    @Test
    void rejectsNegativeAndNonFiniteInputs() {
        assertThrows(IllegalArgumentException.class, () -> BudgetMath.derive(-1.0, 1.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> BudgetMath.derive(1.0, Double.NaN, 1.0));
        assertThrows(IllegalArgumentException.class, () -> BudgetMath.derive(1.0, 1.0, Double.POSITIVE_INFINITY));
    }
}
