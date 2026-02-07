package frc.robot.util.OILayer;

public class ControlCurve {
    private final double ySaturation; // Maximum output, in percentage of possible output
    private final double curvature; // Curvature shift between linear and cubic
    private final double deadzone; // Range of input that will always return zero output
    private final boolean inverted;

    public ControlCurve(double ySaturation, double curvature, double deadzone, boolean inverted) {
        this.ySaturation = ySaturation;
        this.curvature = curvature;
        this.deadzone = deadzone;
        this.inverted = inverted;
    }

    public ControlCurve(double ySaturation, double curvature, double deadzone) {
        this(ySaturation, curvature, deadzone, false);
    }

    public double calculate(double input) {
        /* https://www.desmos.com/calculator/y7nxbdicbj
        First is the deadzone
        y = 0 {|x| < d}
        The second is the curve
        y = -(a * (S * |x + d|)^C) {x < -d}
        or
        y = (a * (S * |x + d|)^C) {x > d}

        Where
        x = input
        y = output
        a = ySaturation
        c = curvature
        d = deadzone
        and 0 <= a,d < 1
        and 0 <= c < 10 (Higher than 10 is usually not needed, but won't break the equation)
        */

        // Apply Deadzone
        if (Math.abs(input) < deadzone) {
            return 0;
        }

        // Calculate Curve
        return (inverted ? -1 : 1)
                * (input < 0 ? -1 : 1)
                * Math.pow(
                        (ySaturation * (1 / (1 - deadzone) * Math.abs(input + (input < 0 ? deadzone : -deadzone)))),
                        1 + curvature);
    }
}
