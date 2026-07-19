package frc.robot.energy;

/** Pure residual-current budget calculations. */
public final class BudgetMath {
    private BudgetMath() {}

    /** Derives the pool available to controlled loads from whole-robot and controlled measurements. */
    public static Result derive(
            double safeRobotCurrentAmps,
            double powerDistributionTotalCurrentAmps,
            double controlledMeasuredCurrentAmps) {
        requireNonnegativeFinite(safeRobotCurrentAmps, "safeRobotCurrentAmps");
        requireNonnegativeFinite(powerDistributionTotalCurrentAmps, "powerDistributionTotalCurrentAmps");
        requireNonnegativeFinite(controlledMeasuredCurrentAmps, "controlledMeasuredCurrentAmps");

        double uncontrolledCurrentAmps =
                Math.max(0.0, powerDistributionTotalCurrentAmps - controlledMeasuredCurrentAmps);
        double controlledPoolAmps = Math.max(0.0, safeRobotCurrentAmps - uncontrolledCurrentAmps);
        double measurementMismatchAmps =
                Math.max(0.0, controlledMeasuredCurrentAmps - powerDistributionTotalCurrentAmps);
        return new Result(uncontrolledCurrentAmps, controlledPoolAmps, measurementMismatchAmps);
    }

    public record Result(double uncontrolledCurrentAmps, double controlledPoolAmps, double measurementMismatchAmps) {}

    private static void requireNonnegativeFinite(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and nonnegative");
        }
    }
}
