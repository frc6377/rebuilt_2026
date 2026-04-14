package frc.robot.util.OILayer;

import static org.junit.jupiter.api.Assertions.*;

import net.jqwik.api.*;
import net.jqwik.api.constraints.DoubleRange;

class ControlCurvePropertyTest {

    private final ControlCurve curve = new ControlCurve(1.0, 0.0, 0.1);

    // ===== Property: Inputs inside deadzone always produce zero =====

    @Property
    void inputsInsideDeadzoneReturnZero(@ForAll @DoubleRange(min = -0.09, max = 0.09) double input) {
        assertEquals(0.0, curve.calculate(input), "Non-zero output for input inside deadzone: " + input);
    }

    // ===== Property: Output is symmetric for non-inverted curves =====

    @Property
    void outputIsSymmetric(@ForAll @DoubleRange(min = 0.0, max = 1.0) double input) {
        double positive = curve.calculate(input);
        double negative = curve.calculate(-input);
        assertEquals(-positive, negative, 1e-9, "Asymmetric output at input=" + input);
    }

    // ===== Property: Output magnitude is monotonically non-decreasing =====

    @Property
    void outputIsMonotonic(
            @ForAll @DoubleRange(min = 0.1, max = 1.0) double a, @ForAll @DoubleRange(min = 0.1, max = 1.0) double b) {
        Assume.that(a > b);
        assertTrue(
                Math.abs(curve.calculate(a)) >= Math.abs(curve.calculate(b)),
                "Monotonicity violated: |f(" + a + ")| < |f(" + b + ")|");
    }

    // ===== Property: Inverted curve is negation of normal curve =====

    @Property
    void invertedIsNegation(@ForAll @DoubleRange(min = -1.0, max = 1.0) double input) {
        ControlCurve normal = new ControlCurve(1.0, 0.0, 0.1, false);
        ControlCurve inverted = new ControlCurve(1.0, 0.0, 0.1, true);
        assertEquals(
                -normal.calculate(input), inverted.calculate(input), 1e-9, "Inverted != -normal at input=" + input);
    }

    // ===== Property: Monotonicity holds across different curvature values =====

    @Property
    void monotonicAcrossCurvatures(
            @ForAll @DoubleRange(min = 0.0, max = 5.0) double curvature,
            @ForAll @DoubleRange(min = 0.11, max = 1.0) double a,
            @ForAll @DoubleRange(min = 0.11, max = 1.0) double b) {
        Assume.that(a > b);
        ControlCurve c = new ControlCurve(1.0, curvature, 0.1);
        assertTrue(
                c.calculate(a) >= c.calculate(b),
                "Monotonicity violated with curvature=" + curvature + ": f(" + a + ") < f(" + b + ")");
    }

    // ===== Property: Output at full input equals ySaturation^(1+curvature) =====

    @Property
    void fullInputProducesExpectedMax(
            @ForAll @DoubleRange(min = 0.1, max = 1.0) double ySat,
            @ForAll @DoubleRange(min = 0.0, max = 3.0) double curvature) {
        ControlCurve c = new ControlCurve(ySat, curvature, 0.0);
        double expected = Math.pow(ySat, 1.0 + curvature);
        assertEquals(expected, c.calculate(1.0), 1e-9, "Max output mismatch for ySat=" + ySat + " curv=" + curvature);
    }
}
