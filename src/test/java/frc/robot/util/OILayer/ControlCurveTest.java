package frc.robot.util.OILayer;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class ControlCurveTest {

    // Standard test curve: linear (curvature=0), 10% deadzone, full saturation
    private final ControlCurve linear = new ControlCurve(1.0, 0.0, 0.1);

    // Curved: quadratic response, 5% deadzone, 80% saturation
    private final ControlCurve curved = new ControlCurve(0.8, 1.0, 0.05);

    // ===== Deadzone Tests =====

    @Test
    void zeroInputReturnsZero() {
        assertEquals(0.0, linear.calculate(0.0));
    }

    @Test
    void inputInsideDeadzoneReturnsZero() {
        assertEquals(0.0, linear.calculate(0.05));
        assertEquals(0.0, linear.calculate(-0.05));
        assertEquals(0.0, linear.calculate(0.099));
        assertEquals(0.0, linear.calculate(-0.099));
    }

    @Test
    void inputAtDeadzoneBoundaryReturnsZero() {
        // Just barely inside deadzone
        assertEquals(0.0, linear.calculate(0.0999));
    }

    @Test
    void inputJustOutsideDeadzoneReturnsNonZero() {
        assertTrue(linear.calculate(0.11) > 0.0);
        assertTrue(linear.calculate(-0.11) < 0.0);
    }

    // ===== Linear Response Tests =====

    @Test
    void fullPositiveInputReturnsOne() {
        // ySat=1.0, curvature=0 → (1.0 * 1.0)^1 = 1.0
        assertEquals(1.0, linear.calculate(1.0), 1e-9);
    }

    @Test
    void fullNegativeInputReturnsNegativeOne() {
        assertEquals(-1.0, linear.calculate(-1.0), 1e-9);
    }

    @Test
    void halfInputLinearResponse() {
        // With deadzone=0.1: normalized = (0.55 - 0.1) / 0.9 = 0.5
        // output = (1.0 * 0.5)^1 = 0.5
        assertEquals(0.5, linear.calculate(0.55), 1e-9);
    }

    // ===== Symmetry Tests =====

    @Test
    void positiveAndNegativeAreSymmetric() {
        double[] inputs = {0.2, 0.5, 0.7, 1.0};
        for (double input : inputs) {
            assertEquals(-linear.calculate(input), linear.calculate(-input), 1e-9, "Symmetry failed at input=" + input);
        }
    }

    // ===== Curved Response Tests =====

    @Test
    void curvedResponseIsLessSensitiveNearZero() {
        // Curved: (0.8 * normalized)^2
        // At input=0.525: normalized = (0.525-0.05)/0.95 = 0.5
        // output = (0.8 * 0.5)^2 = 0.16
        double output = curved.calculate(0.525);
        assertEquals(0.16, output, 1e-6);
    }

    @Test
    void curvedFullInput() {
        // At input=1.0: normalized = (1.0-0.05)/0.95 = 1.0
        // output = (0.8 * 1.0)^2 = 0.64
        assertEquals(0.64, curved.calculate(1.0), 1e-6);
    }

    // ===== Inverted Tests =====

    @Test
    void invertedFlipsSign() {
        ControlCurve inverted = new ControlCurve(1.0, 0.0, 0.1, true);
        assertEquals(-linear.calculate(0.5), inverted.calculate(0.5), 1e-9);
        assertEquals(-linear.calculate(-0.5), inverted.calculate(-0.5), 1e-9);
    }
}
