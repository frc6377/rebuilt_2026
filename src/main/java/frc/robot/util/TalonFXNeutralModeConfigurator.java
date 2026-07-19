package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.time.Duration;
import java.util.Objects;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Applies infrequent TalonFX neutral-mode changes away from the robot loop.
 *
 * <p>The complete startup motor-output group is cloned for every write, so inversion, peaks, and deadband are
 * preserved. The startup baseline is the first pending revision and failed writes retry until superseded or
 * acknowledged.
 */
public final class TalonFXNeutralModeConfigurator implements AutoCloseable {
    private static final Duration DEFAULT_RETRY_DELAY = Duration.ofMillis(100);

    @FunctionalInterface
    interface ConfigApplier {
        StatusCode apply(MotorOutputConfigs config);
    }

    public record Snapshot(
            NeutralModeValue requestedMode,
            long requestedRevision,
            NeutralModeValue lastSuccessfulMode,
            long lastSuccessfulRevision,
            String lastStatusName,
            String lastException,
            long attemptCount,
            long successCount,
            long failureCount,
            boolean pending,
            boolean retrying,
            boolean inFlight,
            boolean workerAlive) {}

    private final ConfigApplier applier;
    private final MotorOutputConfigs baseline;
    private final long retryDelayNanos;
    private final ReentrantLock lock = new ReentrantLock();
    private final Condition stateChanged = lock.newCondition();
    private final Thread workerThread;

    private boolean closed;
    private boolean pending = true;
    private boolean retrying;
    private boolean inFlight;
    private long retryNotBeforeNanos;
    private NeutralModeValue requestedMode;
    private long requestedRevision = 1;
    private NeutralModeValue pendingMode;
    private long pendingRevision = 1;
    private NeutralModeValue lastSuccessfulMode;
    private long lastSuccessfulRevision;
    private String lastStatusName = "";
    private String lastException = "";
    private long attemptCount;
    private long successCount;
    private long failureCount;

    public TalonFXNeutralModeConfigurator(
            String deviceName, TalonFXConfigurator configurator, MotorOutputConfigs baselineMotorOutput) {
        this(deviceName, configurator, baselineMotorOutput, configurator);
    }

    public TalonFXNeutralModeConfigurator(
            String deviceName,
            TalonFXConfigurator configurator,
            MotorOutputConfigs baselineMotorOutput,
            Object deviceConfigurationLock) {
        this(
                deviceName,
                config -> {
                    synchronized (Objects.requireNonNull(deviceConfigurationLock, "deviceConfigurationLock")) {
                        return Objects.requireNonNull(configurator, "configurator")
                                .apply(config);
                    }
                },
                baselineMotorOutput,
                DEFAULT_RETRY_DELAY);
    }

    TalonFXNeutralModeConfigurator(
            String deviceName, ConfigApplier applier, MotorOutputConfigs baselineMotorOutput, Duration retryDelay) {
        String validatedDeviceName = validateDeviceName(deviceName);
        this.applier = Objects.requireNonNull(applier, "applier");
        baseline = Objects.requireNonNull(baselineMotorOutput, "baselineMotorOutput")
                .clone();
        retryDelayNanos = validateDuration(retryDelay);
        requestedMode = baseline.NeutralMode;
        pendingMode = requestedMode;

        workerThread = new Thread(this::runWorker, "TalonFXNeutralModeConfigurator-" + validatedDeviceName);
        workerThread.setDaemon(true);
        workerThread.setPriority(Thread.MIN_PRIORITY);
        workerThread.start();
    }

    public void requestNeutralMode(NeutralModeValue mode) {
        Objects.requireNonNull(mode, "mode");
        lock.lock();
        try {
            if (closed) {
                throw new IllegalStateException("Neutral-mode configurator is closed");
            }
            if (mode == requestedMode) {
                return;
            }
            requestedMode = mode;
            requestedRevision++;
            pendingMode = mode;
            pendingRevision = requestedRevision;
            pending = true;
            retrying = false;
            retryNotBeforeNanos = 0L;
            stateChanged.signalAll();
        } finally {
            lock.unlock();
        }
    }

    public Snapshot snapshot() {
        lock.lock();
        try {
            return new Snapshot(
                    requestedMode,
                    requestedRevision,
                    lastSuccessfulMode,
                    lastSuccessfulRevision,
                    lastStatusName,
                    lastException,
                    attemptCount,
                    successCount,
                    failureCount,
                    pending,
                    retrying,
                    inFlight,
                    workerThread.isAlive());
        } finally {
            lock.unlock();
        }
    }

    boolean awaitIdle(Duration timeout) {
        long remainingNanos = validateDuration(timeout);
        boolean interrupted = false;
        lock.lock();
        try {
            while ((pending || inFlight) && remainingNanos > 0L) {
                try {
                    remainingNanos = stateChanged.awaitNanos(remainingNanos);
                } catch (InterruptedException exception) {
                    interrupted = true;
                    return false;
                }
            }
            return !pending && !inFlight;
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
            closed = true;
            pending = false;
            retrying = false;
            stateChanged.signalAll();
        } finally {
            lock.unlock();
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
                while (!closed && !pending) {
                    stateChanged.await();
                }
                if (closed) {
                    return null;
                }
                long waitNanos = retryNotBeforeNanos - System.nanoTime();
                if (retrying && waitNanos > 0L) {
                    stateChanged.awaitNanos(waitNanos);
                    continue;
                }
                Attempt attempt = new Attempt(pendingRevision, pendingMode);
                pending = false;
                retrying = false;
                inFlight = true;
                return attempt;
            }
        } finally {
            lock.unlock();
        }
    }

    private void apply(Attempt attempt) {
        StatusCode status = null;
        RuntimeException thrown = null;
        try {
            MotorOutputConfigs target = baseline.clone();
            target.NeutralMode = attempt.mode();
            status = applier.apply(target);
        } catch (RuntimeException exception) {
            thrown = exception;
        }

        lock.lock();
        try {
            attemptCount++;
            if (thrown == null && status != null && status.isOK()) {
                successCount++;
                lastSuccessfulMode = attempt.mode();
                lastSuccessfulRevision = attempt.revision();
                lastStatusName = status.getName();
                lastException = "";
            } else {
                failureCount++;
                lastStatusName = status == null ? "" : status.getName();
                lastException = thrown == null
                        ? ""
                        : thrown.getClass().getSimpleName() + ": "
                                + Objects.requireNonNullElse(thrown.getMessage(), "");
                if (!closed && requestedRevision == attempt.revision() && !pending) {
                    pending = true;
                    pendingMode = attempt.mode();
                    pendingRevision = attempt.revision();
                    retrying = true;
                    retryNotBeforeNanos = System.nanoTime() + retryDelayNanos;
                }
            }
            inFlight = false;
            stateChanged.signalAll();
        } finally {
            lock.unlock();
        }
    }

    private static String validateDeviceName(String deviceName) {
        String validated = Objects.requireNonNull(deviceName, "deviceName").strip();
        if (validated.isEmpty()) {
            throw new IllegalArgumentException("deviceName must not be blank");
        }
        return validated;
    }

    private static long validateDuration(Duration duration) {
        Objects.requireNonNull(duration, "duration");
        if (duration.isNegative()) {
            throw new IllegalArgumentException("duration must be nonnegative");
        }
        try {
            return duration.toNanos();
        } catch (ArithmeticException exception) {
            throw new IllegalArgumentException("duration is too large", exception);
        }
    }

    private record Attempt(long revision, NeutralModeValue mode) {}
}
