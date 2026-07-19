package frc.robot.energy;

/**
 * Applies Mechanical Advantage's asymmetric brownout-recovery behavior to a current budget. Reductions are immediate;
 * increases are rate limited for at least a fixed interval after the most recent brownout sample and until the filtered
 * budget regains the post-brownout high-water target.
 */
public final class BrownoutRecoveryLimiter {
    private final double recoveryRateAmpsPerSecond;
    private final double recoveryWindowSeconds;

    private boolean initialized;
    private boolean recovering;
    private double secondsSinceBrownout;
    private double filteredBudgetAmps;
    private double recoveryHighWaterAmps;

    public BrownoutRecoveryLimiter(double recoveryRateAmpsPerSecond, double recoveryWindowSeconds) {
        requireFinitePositive(recoveryRateAmpsPerSecond, "recoveryRateAmpsPerSecond");
        requireFiniteNonnegative(recoveryWindowSeconds, "recoveryWindowSeconds");
        this.recoveryRateAmpsPerSecond = recoveryRateAmpsPerSecond;
        this.recoveryWindowSeconds = recoveryWindowSeconds;
    }

    public void reset(double budgetAmps) {
        requireFiniteNonnegative(budgetAmps, "budgetAmps");
        initialized = true;
        recovering = false;
        secondsSinceBrownout = Double.POSITIVE_INFINITY;
        filteredBudgetAmps = budgetAmps;
        recoveryHighWaterAmps = budgetAmps;
    }

    public double update(double rawBudgetAmps, boolean brownedOut, double dtSeconds) {
        if (!initialized) {
            throw new IllegalStateException("reset must be called before update");
        }
        requireFiniteNonnegative(rawBudgetAmps, "rawBudgetAmps");
        requireFinitePositive(dtSeconds, "dtSeconds");

        if (brownedOut) {
            if (!recovering) {
                recoveryHighWaterAmps = Math.max(filteredBudgetAmps, rawBudgetAmps);
            } else {
                recoveryHighWaterAmps = Math.max(recoveryHighWaterAmps, Math.max(filteredBudgetAmps, rawBudgetAmps));
            }
            recovering = true;
            secondsSinceBrownout = 0.0;
        } else if (recovering) {
            secondsSinceBrownout += dtSeconds;
            recoveryHighWaterAmps = Math.max(recoveryHighWaterAmps, rawBudgetAmps);
        }

        if (rawBudgetAmps <= filteredBudgetAmps || !recovering) {
            filteredBudgetAmps = rawBudgetAmps;
        } else {
            filteredBudgetAmps = Math.min(rawBudgetAmps, filteredBudgetAmps + recoveryRateAmpsPerSecond * dtSeconds);
        }
        if (recovering
                && !brownedOut
                && secondsSinceBrownout >= recoveryWindowSeconds
                && filteredBudgetAmps >= recoveryHighWaterAmps) {
            recovering = false;
        }
        return filteredBudgetAmps;
    }

    public boolean isRecovering() {
        return recovering;
    }

    public double getFilteredBudgetAmps() {
        if (!initialized) {
            throw new IllegalStateException("reset must be called before reading the filtered budget");
        }
        return filteredBudgetAmps;
    }

    private static void requireFinitePositive(double value, String name) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be finite and positive");
        }
    }

    private static void requireFiniteNonnegative(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and nonnegative");
        }
    }
}
