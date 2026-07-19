package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.time.Duration;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class TalonFXNeutralModeConfiguratorTest {
    private static final Duration TEST_TIMEOUT = Duration.ofSeconds(2);

    @Test
    void appliesBaselineThenPreservesTheCompleteGroupWhenChangingNeutralMode() {
        var baseline = baseline();
        var applied = new CopyOnWriteArrayList<MotorOutputConfigs>();
        try (var worker = new TalonFXNeutralModeConfigurator(
                "neutral-test",
                config -> {
                    applied.add(config.clone());
                    return StatusCode.OK;
                },
                baseline,
                Duration.ofMillis(10))) {
            baseline.Inverted = InvertedValue.CounterClockwise_Positive;
            baseline.PeakForwardDutyCycle = 0.1;

            assertTrue(worker.awaitIdle(TEST_TIMEOUT));
            worker.requestNeutralMode(NeutralModeValue.Coast);
            assertTrue(worker.awaitIdle(TEST_TIMEOUT));

            assertEquals(2, applied.size());
            assertMotorOutput(applied.get(0), NeutralModeValue.Brake);
            assertMotorOutput(applied.get(1), NeutralModeValue.Coast);
        }
    }

    @Test
    void retriesFailuresAndKeepsRequestsOffTheCallingThread() throws Exception {
        var attempts = new AtomicInteger();
        var firstAttemptEntered = new CountDownLatch(1);
        var releaseFirstAttempt = new CountDownLatch(1);
        try (var worker = new TalonFXNeutralModeConfigurator(
                "neutral-retry-test",
                config -> {
                    int attempt = attempts.incrementAndGet();
                    if (attempt == 1) {
                        firstAttemptEntered.countDown();
                        await(releaseFirstAttempt);
                        return StatusCode.ConfigFailed;
                    }
                    return StatusCode.OK;
                },
                baseline(),
                Duration.ofMillis(10))) {
            assertTrue(firstAttemptEntered.await(TEST_TIMEOUT.toMillis(), TimeUnit.MILLISECONDS));

            worker.requestNeutralMode(NeutralModeValue.Coast);
            assertTrue(worker.snapshot().workerAlive());
            releaseFirstAttempt.countDown();

            assertTrue(worker.awaitIdle(TEST_TIMEOUT));
            assertTrue(attempts.get() >= 2);
            assertEquals(NeutralModeValue.Coast, worker.snapshot().lastSuccessfulMode());
            assertFalse(worker.snapshot().pending());
        }
    }

    private static MotorOutputConfigs baseline() {
        return new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
                .withPeakForwardDutyCycle(0.8)
                .withPeakReverseDutyCycle(-0.7)
                .withDutyCycleNeutralDeadband(0.03);
    }

    private static void assertMotorOutput(MotorOutputConfigs config, NeutralModeValue expectedMode) {
        assertEquals(InvertedValue.Clockwise_Positive, config.Inverted);
        assertEquals(expectedMode, config.NeutralMode);
        assertEquals(0.8, config.PeakForwardDutyCycle);
        assertEquals(-0.7, config.PeakReverseDutyCycle);
        assertEquals(0.03, config.DutyCycleNeutralDeadband);
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
