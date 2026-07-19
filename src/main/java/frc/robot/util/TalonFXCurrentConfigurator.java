package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import java.time.Duration;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Applies streaming TalonFX supply-current limits away from the robot loop.
 *
 * <p>Each write is rebuilt from a defensive clone of the complete startup current configuration. The worker changes
 * only {@link CurrentLimitsConfigs#SupplyCurrentLimit}, coalesces queued values, and retries failed writes.
 */
public final class TalonFXCurrentConfigurator implements AutoCloseable {
    private static final Duration DEFAULT_RETRY_DELAY = Duration.ofMillis(100);
    private static final Duration DEFAULT_MINIMUM_APPLY_INTERVAL = Duration.ofMillis(100);
    private static final Duration PRODUCTION_STAGGER_STEP = Duration.ofMillis(3);
    private static final int PRODUCTION_STAGGER_SLOTS = 8;
    private static final AtomicInteger nextProductionStaggerSlot = new AtomicInteger();

    @FunctionalInterface
    interface ConfigApplier {
        StatusCode apply(CurrentLimitsConfigs config);
    }

    /** Outcome of the most recent Phoenix configuration attempt. */
    public enum AttemptOutcome {
        NOT_ATTEMPTED,
        SUCCESS,
        STATUS_FAILURE,
        EXCEPTION
    }

    /** Immutable robot-thread telemetry for this asynchronous worker. */
    public record Snapshot(
            String deviceName,
            String workerThreadName,
            double requestedLimitAmps,
            long requestedRevision,
            double lastSuccessfulLimitAmps,
            long lastSuccessfulRevision,
            AttemptOutcome lastOutcome,
            String lastStatusName,
            String lastStatusDescription,
            String lastException,
            double lastAttemptAgeSeconds,
            double lastSuccessfulApplyAgeSeconds,
            long attemptCount,
            long successCount,
            long failureCount,
            long exceptionCount,
            long retryAttemptCount,
            long deduplicatedRequestCount,
            boolean pending,
            boolean retrying,
            boolean inFlight,
            boolean closed,
            boolean workerAlive) {}

    private final String deviceName;
    private final ConfigApplier applier;
    private final CurrentLimitsConfigs baseline;
    private final long retryDelayNanos;
    private final long minimumApplyIntervalNanos;
    private final long nonurgentRequestDelayNanos;

    private final ReentrantLock lock = new ReentrantLock();
    private final Condition stateChanged = lock.newCondition();
    private final Thread workerThread;

    private boolean closed;
    private boolean hasPending;
    private boolean retrying;
    private boolean inFlight;
    private long retryNotBeforeNanos;
    private long pendingNotBeforeNanos;
    private long lastAttemptStartedNanos;
    private long lastSuccessfulApplyNanos;

    private double requestedLimitAmps;
    private long requestedRevision;
    private double pendingLimitAmps;
    private long pendingRevision;

    private double lastSuccessfulLimitAmps = Double.NaN;
    private long lastSuccessfulRevision;
    private AttemptOutcome lastOutcome = AttemptOutcome.NOT_ATTEMPTED;
    private String lastStatusName = "";
    private String lastStatusDescription = "";
    private String lastException = "";
    private long attemptCount;
    private long successCount;
    private long failureCount;
    private long exceptionCount;
    private long retryAttemptCount;
    private long deduplicatedRequestCount;

    /**
     * Creates a worker which uses Phoenix's default configuration timeout.
     *
     * @param deviceName human-readable device name used for telemetry and the worker thread
     * @param configurator Phoenix configurator for one physical TalonFX
     * @param baselineCurrentLimits fully initialized startup current configuration
     */
    public TalonFXCurrentConfigurator(
            String deviceName, TalonFXConfigurator configurator, CurrentLimitsConfigs baselineCurrentLimits) {
        this(deviceName, configurator, baselineCurrentLimits, configurator);
    }

    /**
     * Creates a worker using a caller-supplied lock shared with any other background configuration helper for the same
     * physical device.
     */
    public TalonFXCurrentConfigurator(
            String deviceName,
            TalonFXConfigurator configurator,
            CurrentLimitsConfigs baselineCurrentLimits,
            Object deviceConfigurationLock) {
        this(
                deviceName,
                defaultApplier(configurator, deviceConfigurationLock),
                baselineCurrentLimits,
                DEFAULT_RETRY_DELAY,
                DEFAULT_MINIMUM_APPLY_INTERVAL,
                true,
                nextProductionStagger());
    }

    TalonFXCurrentConfigurator(
            String deviceName,
            ConfigApplier applier,
            CurrentLimitsConfigs baselineCurrentLimits,
            Duration retryDelay,
            Duration minimumApplyInterval) {
        this(deviceName, applier, baselineCurrentLimits, retryDelay, minimumApplyInterval, false, Duration.ZERO);
    }

    TalonFXCurrentConfigurator(
            String deviceName,
            ConfigApplier applier,
            CurrentLimitsConfigs baselineCurrentLimits,
            Duration retryDelay,
            Duration minimumApplyInterval,
            boolean applyBaselineOnStart) {
        this(
                deviceName,
                applier,
                baselineCurrentLimits,
                retryDelay,
                minimumApplyInterval,
                applyBaselineOnStart,
                Duration.ZERO);
    }

    TalonFXCurrentConfigurator(
            String deviceName,
            ConfigApplier applier,
            CurrentLimitsConfigs baselineCurrentLimits,
            Duration retryDelay,
            Duration minimumApplyInterval,
            boolean applyBaselineOnStart,
            Duration nonurgentRequestDelay) {
        this.deviceName = validateDeviceName(deviceName);
        this.applier = Objects.requireNonNull(applier, "applier");
        this.baseline = validateAndCloneBaseline(baselineCurrentLimits);
        this.retryDelayNanos = validateDuration(retryDelay, "retryDelay");
        this.minimumApplyIntervalNanos = validateDuration(minimumApplyInterval, "minimumApplyInterval");
        this.nonurgentRequestDelayNanos = validateDuration(nonurgentRequestDelay, "nonurgentRequestDelay");
        requestedLimitAmps = baseline.SupplyCurrentLimit;
        if (applyBaselineOnStart) {
            requestedRevision = 1;
            pendingLimitAmps = baseline.SupplyCurrentLimit;
            pendingRevision = requestedRevision;
            hasPending = true;
            pendingNotBeforeNanos = saturatingAdd(System.nanoTime(), nonurgentRequestDelayNanos);
        }

        workerThread = new Thread(this::runWorker, "TalonFXCurrentConfigurator-" + this.deviceName);
        workerThread.setDaemon(true);
        workerThread.setPriority(Thread.MIN_PRIORITY);
        workerThread.start();
    }

    /**
     * Queues a new per-motor supply-current limit.
     *
     * <p>Only the newest distinct value is retained. Repeating a value which is waiting for a retry does not cancel
     * that retry.
     */
    public void requestSupplyCurrentLimit(double limitAmps) {
        validateLimit(limitAmps);

        lock.lock();
        try {
            if (closed) {
                throw new IllegalStateException("Current configurator is closed");
            }
            if (Double.compare(limitAmps, requestedLimitAmps) == 0) {
                deduplicatedRequestCount++;
                return;
            }

            boolean requestAlreadyPending = hasPending;
            requestedLimitAmps = limitAmps;
            requestedRevision++;
            pendingLimitAmps = limitAmps;
            pendingRevision = requestedRevision;
            hasPending = true;
            if (!requestAlreadyPending) {
                pendingNotBeforeNanos = saturatingAdd(System.nanoTime(), nonurgentRequestDelayNanos);
            }

            // A distinct revision supersedes any delayed retry of the previous value.
            retrying = false;
            retryNotBeforeNanos = 0L;
            stateChanged.signalAll();
        } finally {
            lock.unlock();
        }
    }

    /** Returns an immutable snapshot suitable for logging from the robot thread. */
    public Snapshot snapshot() {
        lock.lock();
        try {
            long nowNanos = System.nanoTime();
            return new Snapshot(
                    deviceName,
                    workerThread.getName(),
                    requestedLimitAmps,
                    requestedRevision,
                    lastSuccessfulLimitAmps,
                    lastSuccessfulRevision,
                    lastOutcome,
                    lastStatusName,
                    lastStatusDescription,
                    lastException,
                    ageSeconds(nowNanos, lastAttemptStartedNanos),
                    ageSeconds(nowNanos, lastSuccessfulApplyNanos),
                    attemptCount,
                    successCount,
                    failureCount,
                    exceptionCount,
                    retryAttemptCount,
                    deduplicatedRequestCount,
                    hasPending,
                    retrying,
                    inFlight,
                    closed,
                    workerThread.isAlive());
        } finally {
            lock.unlock();
        }
    }

    boolean awaitIdle(Duration timeout) {
        long timeoutNanos = validateDuration(timeout, "timeout");
        boolean interrupted = false;
        lock.lock();
        try {
            long remainingNanos = timeoutNanos;
            while ((hasPending || inFlight) && remainingNanos > 0L) {
                try {
                    remainingNanos = stateChanged.awaitNanos(remainingNanos);
                } catch (InterruptedException exception) {
                    interrupted = true;
                    return false;
                }
            }
            return !hasPending && !inFlight;
        } finally {
            lock.unlock();
            if (interrupted) {
                Thread.currentThread().interrupt();
            }
        }
    }

    @Override
    public void close() {
        lock.lock();
        try {
            if (!closed) {
                closed = true;
                hasPending = false;
                retrying = false;
                stateChanged.signalAll();
            }
        } finally {
            lock.unlock();
        }

        if (Thread.currentThread() == workerThread) {
            return;
        }

        workerThread.interrupt();
        boolean interrupted = false;
        while (workerThread.isAlive()) {
            try {
                workerThread.join();
            } catch (InterruptedException exception) {
                interrupted = true;
            }
        }
        if (interrupted) {
            Thread.currentThread().interrupt();
        }
    }

    private void runWorker() {
        try {
            while (true) {
                Attempt attempt = waitForAttempt();
                if (attempt == null) {
                    return;
                }
                apply(attempt);
            }
        } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
        } finally {
            lock.lock();
            try {
                inFlight = false;
                stateChanged.signalAll();
            } finally {
                lock.unlock();
            }
        }
    }

    private Attempt waitForAttempt() throws InterruptedException {
        lock.lockInterruptibly();
        try {
            while (true) {
                while (!closed && !hasPending) {
                    stateChanged.await();
                }
                if (closed) {
                    return null;
                }

                long nowNanos = System.nanoTime();
                long waitNanos = nanosUntilEligible(nowNanos);
                if (waitNanos > 0L) {
                    stateChanged.awaitNanos(waitNanos);
                    continue;
                }

                Attempt attempt = new Attempt(pendingRevision, pendingLimitAmps, retrying);
                hasPending = false;
                retrying = false;
                inFlight = true;
                lastAttemptStartedNanos = nowNanos;
                return attempt;
            }
        } finally {
            lock.unlock();
        }
    }

    private long nanosUntilEligible(long nowNanos) {
        long eligibleNanos = retrying ? retryNotBeforeNanos : 0L;
        double effectiveLimit =
                Double.isFinite(lastSuccessfulLimitAmps) ? lastSuccessfulLimitAmps : baseline.SupplyCurrentLimit;
        boolean urgentDecrease = pendingLimitAmps < effectiveLimit;
        if (!urgentDecrease) {
            eligibleNanos = Math.max(eligibleNanos, pendingNotBeforeNanos);
            if (lastAttemptStartedNanos != 0L) {
                eligibleNanos =
                        Math.max(eligibleNanos, saturatingAdd(lastAttemptStartedNanos, minimumApplyIntervalNanos));
            }
        }
        return Math.max(0L, eligibleNanos - nowNanos);
    }

    private void apply(Attempt attempt) {
        StatusCode status = null;
        RuntimeException thrownException = null;
        try {
            CurrentLimitsConfigs target = baseline.clone();
            target.SupplyCurrentLimit = attempt.limitAmps();
            status = applier.apply(target);
        } catch (RuntimeException exception) {
            thrownException = exception;
        }

        lock.lock();
        try {
            attemptCount++;
            if (attempt.retryAttempt()) {
                retryAttemptCount++;
            }

            if (thrownException != null) {
                exceptionCount++;
                recordFailure(
                        attempt,
                        AttemptOutcome.EXCEPTION,
                        "",
                        "",
                        thrownException.getClass().getSimpleName() + ": " + safeMessage(thrownException));
            } else if (status == null || !status.isOK()) {
                String statusName = status == null ? "NullStatusCode" : status.getName();
                String statusDescription = status == null ? "ConfigApplier returned null" : status.getDescription();
                recordFailure(attempt, AttemptOutcome.STATUS_FAILURE, statusName, statusDescription, "");
            } else {
                successCount++;
                lastSuccessfulLimitAmps = attempt.limitAmps();
                lastSuccessfulRevision = attempt.revision();
                lastSuccessfulApplyNanos = System.nanoTime();
                lastOutcome = AttemptOutcome.SUCCESS;
                lastStatusName = status.getName();
                lastStatusDescription = status.getDescription();
                lastException = "";
            }

            inFlight = false;
            stateChanged.signalAll();
        } finally {
            lock.unlock();
        }
    }

    private void recordFailure(
            Attempt attempt,
            AttemptOutcome outcome,
            String statusName,
            String statusDescription,
            String exceptionDescription) {
        failureCount++;
        lastOutcome = outcome;
        lastStatusName = statusName;
        lastStatusDescription = statusDescription;
        lastException = exceptionDescription;

        if (!closed && requestedRevision == attempt.revision() && !hasPending) {
            pendingRevision = attempt.revision();
            pendingLimitAmps = attempt.limitAmps();
            hasPending = true;
            retrying = true;
            retryNotBeforeNanos = saturatingAdd(System.nanoTime(), retryDelayNanos);
        }
    }

    private static ConfigApplier defaultApplier(TalonFXConfigurator configurator, Object deviceConfigurationLock) {
        Objects.requireNonNull(configurator, "configurator");
        Objects.requireNonNull(deviceConfigurationLock, "deviceConfigurationLock");
        // Deliberately use the no-timeout overload. Phoenix's default timeout behaves reliably for repeated configs.
        return config -> {
            synchronized (deviceConfigurationLock) {
                return configurator.apply(config);
            }
        };
    }

    private static Duration nextProductionStagger() {
        int slot = Math.floorMod(nextProductionStaggerSlot.getAndIncrement(), PRODUCTION_STAGGER_SLOTS);
        return PRODUCTION_STAGGER_STEP.multipliedBy(slot);
    }

    private static String validateDeviceName(String deviceName) {
        Objects.requireNonNull(deviceName, "deviceName");
        String stripped = deviceName.strip();
        if (stripped.isEmpty()) {
            throw new IllegalArgumentException("deviceName must not be blank");
        }
        return stripped;
    }

    private static CurrentLimitsConfigs validateAndCloneBaseline(CurrentLimitsConfigs baselineCurrentLimits) {
        Objects.requireNonNull(baselineCurrentLimits, "baselineCurrentLimits");
        requireFinite(baselineCurrentLimits.StatorCurrentLimit, "baseline StatorCurrentLimit");
        requireFinite(baselineCurrentLimits.SupplyCurrentLimit, "baseline SupplyCurrentLimit");
        requireFinite(baselineCurrentLimits.SupplyCurrentLowerLimit, "baseline SupplyCurrentLowerLimit");
        requireFinite(baselineCurrentLimits.SupplyCurrentLowerTime, "baseline SupplyCurrentLowerTime");
        validateLimit(baselineCurrentLimits.SupplyCurrentLimit);
        return baselineCurrentLimits.clone();
    }

    private static long validateDuration(Duration duration, String name) {
        Objects.requireNonNull(duration, name);
        if (duration.isNegative()) {
            throw new IllegalArgumentException(name + " must be nonnegative");
        }
        try {
            return duration.toNanos();
        } catch (ArithmeticException exception) {
            throw new IllegalArgumentException(name + " is too large", exception);
        }
    }

    private static void validateLimit(double limitAmps) {
        if (!Double.isFinite(limitAmps) || limitAmps <= 0.0) {
            throw new IllegalArgumentException("Supply-current limit must be finite and greater than zero");
        }
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
    }

    private static String safeMessage(RuntimeException exception) {
        return exception.getMessage() == null ? "" : exception.getMessage();
    }

    private static long saturatingAdd(long left, long right) {
        if (right > 0L && left > Long.MAX_VALUE - right) {
            return Long.MAX_VALUE;
        }
        return left + right;
    }

    private static double ageSeconds(long nowNanos, long eventNanos) {
        return eventNanos == 0L ? Double.NaN : Math.max(0.0, (nowNanos - eventNanos) / 1_000_000_000.0);
    }

    private record Attempt(long revision, double limitAmps, boolean retryAttempt) {}
}
