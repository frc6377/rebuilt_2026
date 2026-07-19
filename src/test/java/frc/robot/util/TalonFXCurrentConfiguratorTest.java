package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import java.time.Duration;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class TalonFXCurrentConfiguratorTest {
    private static final Duration TEST_TIMEOUT = Duration.ofSeconds(2);

    private final List<TalonFXCurrentConfigurator> workers = new CopyOnWriteArrayList<>();

    @AfterEach
    void closeWorkers() {
        workers.forEach(TalonFXCurrentConfigurator::close);
    }

    @Test
    void preservesEveryBaselineFieldAndDefensivelyClones() {
        var baseline = baseline();
        var applied = new CopyOnWriteArrayList<CurrentLimitsConfigs>();
        var worker = worker(
                config -> {
                    applied.add(config.clone());
                    config.StatorCurrentLimit = -1.0;
                    config.StatorCurrentLimitEnable = false;
                    config.SupplyCurrentLimit = -2.0;
                    config.SupplyCurrentLimitEnable = false;
                    config.SupplyCurrentLowerLimit = -3.0;
                    config.SupplyCurrentLowerTime = -4.0;
                    return StatusCode.OK;
                },
                Duration.ZERO,
                Duration.ZERO,
                baseline);

        baseline.StatorCurrentLimit = 999.0;
        baseline.SupplyCurrentLowerLimit = 998.0;

        worker.requestSupplyCurrentLimit(31.0);
        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
        worker.requestSupplyCurrentLimit(32.0);
        assertTrue(worker.awaitIdle(TEST_TIMEOUT));

        assertEquals(2, applied.size());
        assertCurrentLimits(applied.get(0), 31.0);
        assertCurrentLimits(applied.get(1), 32.0);
    }

    @Test
    void appliesAndRetriesTheBaselineOnProductionStartup() {
        var attempts = new AtomicInteger();
        var appliedLimits = new CopyOnWriteArrayList<Double>();
        var worker = new TalonFXCurrentConfigurator(
                "startup-test",
                config -> {
                    appliedLimits.add(config.SupplyCurrentLimit);
                    return attempts.incrementAndGet() == 1 ? StatusCode.ConfigFailed : StatusCode.OK;
                },
                baseline(),
                Duration.ofMillis(20),
                Duration.ZERO,
                true);
        workers.add(worker);

        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
        assertAll(
                () -> assertEquals(List.of(50.0, 50.0), appliedLimits),
                () -> assertEquals(1, worker.snapshot().lastSuccessfulRevision()),
                () -> assertEquals(50.0, worker.snapshot().lastSuccessfulLimitAmps()),
                () -> assertEquals(1, worker.snapshot().failureCount()),
                () -> assertEquals(1, worker.snapshot().retryAttemptCount()));
    }

    @Test
    void deduplicatesEqualRequests() throws Exception {
        var attemptCount = new AtomicInteger();
        var worker = worker(config -> {
            attemptCount.incrementAndGet();
            return StatusCode.OK;
        });

        worker.requestSupplyCurrentLimit(30.0);
        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
        worker.requestSupplyCurrentLimit(30.0);
        TimeUnit.MILLISECONDS.sleep(40);

        assertAll(
                () -> assertEquals(1, attemptCount.get()),
                () -> assertEquals(1, worker.snapshot().deduplicatedRequestCount()),
                () -> assertFalse(worker.snapshot().pending()));
    }

    @Test
    void reportsMonotonicAttemptAndSuccessfulApplyAges() throws Exception {
        var worker = worker(config -> StatusCode.OK);

        assertAll(
                () -> assertTrue(Double.isNaN(worker.snapshot().lastAttemptAgeSeconds())),
                () -> assertTrue(Double.isNaN(worker.snapshot().lastSuccessfulApplyAgeSeconds())));

        worker.requestSupplyCurrentLimit(30.0);
        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
        var firstSnapshot = worker.snapshot();
        TimeUnit.MILLISECONDS.sleep(5);
        var laterSnapshot = worker.snapshot();

        assertAll(
                () -> assertTrue(firstSnapshot.lastAttemptAgeSeconds() >= 0.0),
                () -> assertTrue(firstSnapshot.lastSuccessfulApplyAgeSeconds() >= 0.0),
                () -> assertTrue(laterSnapshot.lastAttemptAgeSeconds() >= firstSnapshot.lastAttemptAgeSeconds()),
                () -> assertTrue(laterSnapshot.lastSuccessfulApplyAgeSeconds()
                        >= firstSnapshot.lastSuccessfulApplyAgeSeconds()));
    }

    @Test
    void appliesOnlyLatestValueQueuedWhileAnotherValueIsInFlight() throws Exception {
        var firstAttemptEntered = new CountDownLatch(1);
        var releaseFirstAttempt = new CountDownLatch(1);
        var appliedLimits = new CopyOnWriteArrayList<Double>();
        var worker = worker(
                config -> {
                    appliedLimits.add(config.SupplyCurrentLimit);
                    if (appliedLimits.size() == 1) {
                        firstAttemptEntered.countDown();
                        await(releaseFirstAttempt);
                    }
                    return StatusCode.OK;
                },
                Duration.ZERO,
                Duration.ZERO);

        worker.requestSupplyCurrentLimit(10.0);
        assertTrue(firstAttemptEntered.await(TEST_TIMEOUT.toMillis(), TimeUnit.MILLISECONDS));
        worker.requestSupplyCurrentLimit(20.0);
        worker.requestSupplyCurrentLimit(30.0);
        releaseFirstAttempt.countDown();

        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
        assertEquals(List.of(10.0, 30.0), appliedLimits);
    }

    @Test
    void retriesStatusFailuresAfterConfiguredDelay() {
        var attempts = new AtomicInteger();
        var firstAttemptNanos = new AtomicLong();
        var secondAttemptNanos = new AtomicLong();
        var worker = worker(
                config -> {
                    int attempt = attempts.incrementAndGet();
                    if (attempt == 1) {
                        firstAttemptNanos.set(System.nanoTime());
                        return StatusCode.ConfigFailed;
                    }
                    secondAttemptNanos.set(System.nanoTime());
                    return StatusCode.OK;
                },
                Duration.ofMillis(20),
                Duration.ZERO);

        worker.requestSupplyCurrentLimit(25.0);

        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
        assertAll(
                () -> assertEquals(2, attempts.get()),
                () -> assertTrue(secondAttemptNanos.get() - firstAttemptNanos.get()
                        >= Duration.ofMillis(15).toNanos()),
                () -> assertEquals(1, worker.snapshot().failureCount()),
                () -> assertEquals(1, worker.snapshot().retryAttemptCount()),
                () -> assertEquals(
                        TalonFXCurrentConfigurator.AttemptOutcome.SUCCESS,
                        worker.snapshot().lastOutcome()));
    }

    @Test
    void sustainedFailureNeverReportsAnUnacknowledgedLimitAsSuccessful() {
        var attempts = new AtomicInteger();
        var worker = worker(
                config -> {
                    attempts.incrementAndGet();
                    return StatusCode.ConfigFailed;
                },
                Duration.ofMillis(10),
                Duration.ZERO);

        worker.requestSupplyCurrentLimit(25.0);

        assertTrue(waitUntil(() -> attempts.get() >= 3, TEST_TIMEOUT));
        var snapshot = worker.snapshot();
        assertAll(
                () -> assertTrue(Double.isNaN(snapshot.lastSuccessfulLimitAmps())),
                () -> assertEquals(0, snapshot.lastSuccessfulRevision()),
                () -> assertEquals(0, snapshot.successCount()),
                () -> assertTrue(snapshot.failureCount() >= 3),
                () -> assertTrue(snapshot.workerAlive()),
                () -> assertTrue(snapshot.pending() || snapshot.retrying() || snapshot.inFlight()));
    }

    @Test
    void newerRequestSupersedesScheduledRetry() {
        var appliedLimits = new CopyOnWriteArrayList<Double>();
        var worker = worker(
                config -> {
                    appliedLimits.add(config.SupplyCurrentLimit);
                    return appliedLimits.size() == 1 ? StatusCode.ConfigFailed : StatusCode.OK;
                },
                Duration.ofSeconds(1),
                Duration.ZERO);

        worker.requestSupplyCurrentLimit(25.0);
        assertTrue(waitUntil(
                () -> worker.snapshot().retrying() && worker.snapshot().failureCount() == 1, TEST_TIMEOUT));

        worker.requestSupplyCurrentLimit(35.0);

        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
        assertEquals(List.of(25.0, 35.0), appliedLimits);
    }

    @Test
    void equalRequestDoesNotCancelScheduledRetry() {
        var attempts = new AtomicInteger();
        var worker = worker(
                config -> attempts.incrementAndGet() == 1 ? StatusCode.ConfigFailed : StatusCode.OK,
                Duration.ofMillis(40),
                Duration.ZERO);

        worker.requestSupplyCurrentLimit(25.0);
        assertTrue(waitUntil(() -> worker.snapshot().retrying(), TEST_TIMEOUT));
        worker.requestSupplyCurrentLimit(25.0);

        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
        assertAll(
                () -> assertEquals(2, attempts.get()),
                () -> assertEquals(1, worker.snapshot().deduplicatedRequestCount()),
                () -> assertEquals(1, worker.snapshot().retryAttemptCount()));
    }

    @Test
    void survivesRuntimeExceptionAndRetries() {
        var attempts = new AtomicInteger();
        var worker = worker(
                config -> {
                    if (attempts.incrementAndGet() == 1) {
                        throw new IllegalStateException("synthetic apply failure");
                    }
                    return StatusCode.OK;
                },
                Duration.ofMillis(20),
                Duration.ZERO);

        worker.requestSupplyCurrentLimit(25.0);

        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
        assertAll(
                () -> assertEquals(2, attempts.get()),
                () -> assertEquals(1, worker.snapshot().failureCount()),
                () -> assertEquals(1, worker.snapshot().exceptionCount()),
                () -> assertTrue(worker.snapshot().workerAlive()));
    }

    @Test
    void urgentDecreaseBypassesMinimumApplyInterval() {
        var appliedLimits = new CopyOnWriteArrayList<Double>();
        var worker = worker(
                config -> {
                    appliedLimits.add(config.SupplyCurrentLimit);
                    return StatusCode.OK;
                },
                Duration.ofMillis(20),
                Duration.ofSeconds(1));

        worker.requestSupplyCurrentLimit(60.0);
        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
        worker.requestSupplyCurrentLimit(70.0);
        assertTrue(waitUntil(() -> worker.snapshot().pending(), TEST_TIMEOUT));
        long decreaseRequestedNanos = System.nanoTime();
        worker.requestSupplyCurrentLimit(40.0);

        assertTrue(worker.awaitIdle(Duration.ofMillis(400)));
        assertAll(
                () -> assertEquals(List.of(60.0, 40.0), appliedLimits),
                () -> assertTrue(System.nanoTime() - decreaseRequestedNanos
                        < Duration.ofMillis(400).toNanos()));
    }

    @Test
    void nonurgentIncreaseHonorsItsConfiguredDeviceStagger() throws Exception {
        var attemptEntered = new CountDownLatch(1);
        var worker = new TalonFXCurrentConfigurator(
                "stagger-test",
                config -> {
                    attemptEntered.countDown();
                    return StatusCode.OK;
                },
                baseline(),
                Duration.ofMillis(20),
                Duration.ZERO,
                false,
                Duration.ofMillis(80));
        workers.add(worker);

        worker.requestSupplyCurrentLimit(60.0);

        assertFalse(attemptEntered.await(30, TimeUnit.MILLISECONDS));
        assertTrue(attemptEntered.await(TEST_TIMEOUT.toMillis(), TimeUnit.MILLISECONDS));
        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
    }

    @Test
    void supersedingIncreasesDoNotRestartTheConfiguredDeviceStagger() throws Exception {
        var attemptEntered = new CountDownLatch(1);
        var worker = new TalonFXCurrentConfigurator(
                "superseded-stagger-test",
                config -> {
                    attemptEntered.countDown();
                    return StatusCode.OK;
                },
                baseline(),
                Duration.ofMillis(20),
                Duration.ZERO,
                false,
                Duration.ofMillis(80));
        workers.add(worker);

        worker.requestSupplyCurrentLimit(60.0);
        for (int i = 0; i < 6; i++) {
            TimeUnit.MILLISECONDS.sleep(25);
            worker.requestSupplyCurrentLimit(61.0 + i);
        }

        assertTrue(attemptEntered.await(20, TimeUnit.MILLISECONDS));
        assertTrue(worker.awaitIdle(TEST_TIMEOUT));
    }

    @Test
    void workerThreadIsDaemonAndLowPriorityAndCloseIsIdempotent() {
        var worker = worker(config -> StatusCode.OK);
        var snapshot = worker.snapshot();
        Thread thread = Thread.getAllStackTraces().keySet().stream()
                .filter(candidate -> candidate.getName().equals(snapshot.workerThreadName()))
                .findFirst()
                .orElse(null);

        assertNotNull(thread);
        assertAll(
                () -> assertTrue(thread.isDaemon()),
                () -> assertEquals(Thread.MIN_PRIORITY, thread.getPriority()),
                () -> assertTrue(snapshot.workerAlive()));

        worker.close();
        worker.close();

        assertFalse(worker.snapshot().workerAlive());
        assertThrows(IllegalStateException.class, () -> worker.requestSupplyCurrentLimit(20.0));
    }

    @Test
    void rejectsInvalidLimits() {
        var worker = worker(config -> StatusCode.OK);

        assertAll(
                () -> assertThrows(IllegalArgumentException.class, () -> worker.requestSupplyCurrentLimit(0.0)),
                () -> assertThrows(IllegalArgumentException.class, () -> worker.requestSupplyCurrentLimit(-1.0)),
                () -> assertThrows(IllegalArgumentException.class, () -> worker.requestSupplyCurrentLimit(Double.NaN)),
                () -> assertThrows(
                        IllegalArgumentException.class,
                        () -> worker.requestSupplyCurrentLimit(Double.POSITIVE_INFINITY)));
    }

    private TalonFXCurrentConfigurator worker(TalonFXCurrentConfigurator.ConfigApplier applier) {
        return worker(applier, Duration.ofMillis(20), Duration.ofMillis(20));
    }

    private TalonFXCurrentConfigurator worker(
            TalonFXCurrentConfigurator.ConfigApplier applier, Duration retryDelay, Duration minimumApplyInterval) {
        return worker(applier, retryDelay, minimumApplyInterval, baseline());
    }

    private TalonFXCurrentConfigurator worker(
            TalonFXCurrentConfigurator.ConfigApplier applier,
            Duration retryDelay,
            Duration minimumApplyInterval,
            CurrentLimitsConfigs baseline) {
        var worker = new TalonFXCurrentConfigurator("test-device", applier, baseline, retryDelay, minimumApplyInterval);
        workers.add(worker);
        return worker;
    }

    private static CurrentLimitsConfigs baseline() {
        return new CurrentLimitsConfigs()
                .withStatorCurrentLimit(77.0)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(34.0)
                .withSupplyCurrentLowerTime(0.75);
    }

    private static void assertCurrentLimits(CurrentLimitsConfigs config, double expectedSupplyLimit) {
        assertAll(
                () -> assertEquals(77.0, config.StatorCurrentLimit),
                () -> assertTrue(config.StatorCurrentLimitEnable),
                () -> assertEquals(expectedSupplyLimit, config.SupplyCurrentLimit),
                () -> assertTrue(config.SupplyCurrentLimitEnable),
                () -> assertEquals(34.0, config.SupplyCurrentLowerLimit),
                () -> assertEquals(0.75, config.SupplyCurrentLowerTime));
    }

    private static boolean waitUntil(BooleanSupplier condition, Duration timeout) {
        long deadline = System.nanoTime() + timeout.toNanos();
        while (System.nanoTime() < deadline) {
            if (condition.getAsBoolean()) {
                return true;
            }
            try {
                TimeUnit.MILLISECONDS.sleep(2);
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
                return false;
            }
        }
        return condition.getAsBoolean();
    }

    private static void await(CountDownLatch latch) {
        try {
            if (!latch.await(TEST_TIMEOUT.toMillis(), TimeUnit.MILLISECONDS)) {
                throw new IllegalStateException("Timed out waiting for test latch");
            }
        } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException("Interrupted waiting for test latch", exception);
        }
    }
}
