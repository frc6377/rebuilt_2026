package frc.robot.energy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.energy.PowerBudgetManager.ModelBudget;
import frc.robot.energy.StateAwareCurrentAllocator.Activity;
import frc.robot.energy.StateAwareCurrentAllocator.AllocationConfig;
import frc.robot.energy.StateAwareCurrentAllocator.LoadPolicy;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;

class PowerBudgetManagerTest {
    private static final double EPSILON = 1e-9;

    private static final AllocationConfig ALLOCATION_CONFIG = new AllocationConfig(
            new LoadPolicy(4, 5.0, 10.0, 10.0, 30.0),
            new LoadPolicy(2, 2.0, 8.0, 12.0, 20.0),
            new LoadPolicy(1, 2.0, 8.0, 12.0, 15.0),
            new LoadPolicy(1, 2.0, 10.0, 15.0, 20.0),
            0.5);

    @Test
    void subtractsUncontrolledCurrentAndOnlyCalculatesInShadow() {
        FakePowerIO power = new FakePowerIO(12.0, 180.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = 60.0;
        loads.rollerCurrent = 20.0;
        loads.extenderCurrent = 10.0;
        loads.indexerCurrent = 10.0;

        PowerBudgetManager manager = createManager(
                power, loads, new ConstantBudgetSource(150.0), PowerBudgetManager.Mode.SHADOW, 1_000.0, 0.0);

        PowerBudgetManager.Snapshot snapshot = manager.updateForTest(0.02, true);

        assertTrue(snapshot.inputsValid());
        assertEquals(100.0, snapshot.controlledCurrentAmps(), EPSILON);
        assertEquals(80.0, snapshot.uncontrolledCurrentAmps(), EPSILON);
        assertEquals(70.0, snapshot.rawControlledPoolAmps(), EPSILON);
        assertEquals(70.0, snapshot.filteredControlledPoolAmps(), EPSILON);
        assertEquals(0, loads.totalLimitRequestCount());
    }

    @Test
    void clampsRegenerativeControlledSamplesAtZeroPerGroup() {
        FakePowerIO power = new FakePowerIO(12.0, 20.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = -5.0;
        loads.rollerCurrent = 10.0;

        PowerBudgetManager manager = createManager(
                power, loads, new ConstantBudgetSource(100.0), PowerBudgetManager.Mode.SHADOW, 1_000.0, 0.0);

        PowerBudgetManager.Snapshot snapshot = manager.updateForTest(0.02, true);

        assertTrue(snapshot.inputsValid());
        assertEquals(10.0, snapshot.controlledCurrentAmps(), EPSILON);
        assertEquals(10.0, snapshot.uncontrolledCurrentAmps(), EPSILON);
        assertEquals(90.0, snapshot.rawControlledPoolAmps(), EPSILON);
    }

    @Test
    void sendsTypedPriorityAllocationOnlyInActiveMode() {
        FakePowerIO power = new FakePowerIO(12.0, 60.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = 30.0;
        loads.rollerCurrent = 15.0;
        loads.extenderCurrent = 7.5;
        loads.indexerCurrent = 7.5;
        loads.activity = new Activity(true, true, true);

        PowerBudgetManager manager = createManager(
                power, loads, new ConstantBudgetSource(60.0), PowerBudgetManager.Mode.ACTIVE, 1_000.0, 0.0);

        PowerBudgetManager.Snapshot snapshot = manager.updateForTest(0.02, true);

        assertEquals(10.0, snapshot.allocation().indexerPerMotor(), EPSILON);
        assertEquals(8.0, snapshot.allocation().extenderPerMotor(), EPSILON);
        assertEquals(8.0, snapshot.allocation().rollersPerMotor(), EPSILON);
        assertEquals(6.5, snapshot.allocation().drivePerMotor(), EPSILON);
        assertEquals(1, loads.driveLimitRequests);
        assertEquals(1, loads.rollerLimitRequests);
        assertEquals(1, loads.extenderLimitRequests);
        assertEquals(1, loads.indexerLimitRequests);
        assertEquals(snapshot.allocation().drivePerMotor(), loads.lastDriveLimit, EPSILON);
        assertEquals(snapshot.allocation().rollersPerMotor(), loads.lastRollerLimit, EPSILON);
        assertEquals(snapshot.allocation().extenderPerMotor(), loads.lastExtenderLimit, EPSILON);
        assertEquals(snapshot.allocation().indexerPerMotor(), loads.lastIndexerLimit, EPSILON);
    }

    @Test
    void activeMinimumDeficitOutranksStandbyDeficitInTheLimitingReason() {
        FakePowerIO power = new FakePowerIO(12.0, 0.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.activity = new Activity(false, false, true);
        PowerBudgetManager manager = createManager(
                power, loads, new ConstantBudgetSource(0.0), PowerBudgetManager.Mode.SHADOW, 1_000.0, 0.0);

        PowerBudgetManager.Snapshot snapshot = manager.updateForTest(0.02, true);

        assertEquals(PowerBudgetManager.LimitingReason.ACTIVE_MINIMUM_DEFICIT, snapshot.limitingReason());
    }

    @Test
    void restoresStartupLimitsOnceWhenLeavingActive() {
        FakePowerIO power = new FakePowerIO(12.0, 60.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = 60.0;
        AtomicReference<PowerBudgetManager.Mode> mode = new AtomicReference<>(PowerBudgetManager.Mode.ACTIVE);
        PowerBudgetManager manager = createManager(power, loads, new ConstantBudgetSource(100.0), mode, 1_000.0, 0.0);

        manager.updateForTest(0.02, true);
        mode.set(PowerBudgetManager.Mode.SHADOW);
        PowerBudgetManager.Snapshot transition = manager.updateForTest(0.02, true);
        manager.updateForTest(0.02, true);

        assertTrue(transition.startupLimitsRestored());
        assertEquals(2, loads.driveLimitRequests);
        assertEquals(2, loads.rollerLimitRequests);
        assertEquals(2, loads.extenderLimitRequests);
        assertEquals(2, loads.indexerLimitRequests);
        assertEquals(EnergyConstants.DRIVE_STARTUP_LIMIT_AMPS, loads.lastDriveLimit, EPSILON);
        assertEquals(EnergyConstants.ROLLER_STARTUP_LIMIT_AMPS, loads.lastRollerLimit, EPSILON);
        assertEquals(EnergyConstants.EXTENDER_STARTUP_LIMIT_AMPS, loads.lastExtenderLimit, EPSILON);
        assertEquals(EnergyConstants.INDEXER_STARTUP_LIMIT_AMPS, loads.lastIndexerLimit, EPSILON);
    }

    @Test
    void restoresStartupLimitsAfterInvalidPowerSample() {
        FakePowerIO power = new FakePowerIO(12.0, 60.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = 60.0;
        PowerBudgetManager manager = createManager(
                power, loads, new ConstantBudgetSource(100.0), PowerBudgetManager.Mode.ACTIVE, 1_000.0, 0.0);

        manager.updateForTest(0.02, true);
        power.totalCurrentAmps = Double.NaN;
        PowerBudgetManager.Snapshot invalid = advanceUntilStartupLimitsRestore(manager);

        assertFalse(invalid.inputsValid());
        assertEquals(PowerBudgetManager.LimitingReason.INVALID_INPUT, invalid.limitingReason());
        assertTrue(invalid.startupLimitsRestored());
        assertEquals(2, loads.driveLimitRequests);
        assertEquals(EnergyConstants.DRIVE_STARTUP_LIMIT_AMPS, loads.lastDriveLimit, EPSILON);
    }

    @Test
    void transientInvalidSampleHoldsTheLastDynamicLimitsWithoutAWriteBurst() {
        FakePowerIO power = new FakePowerIO(12.0, 60.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = 60.0;
        PowerBudgetManager manager = createManager(
                power, loads, new ConstantBudgetSource(100.0), PowerBudgetManager.Mode.ACTIVE, 1_000.0, 0.0);

        manager.updateForTest(0.02, true);
        power.totalCurrentAmps = Double.NaN;
        PowerBudgetManager.Snapshot transientFailure = manager.updateForTest(0.02, true);

        assertFalse(transientFailure.inputsValid());
        assertFalse(transientFailure.startupLimitsRestored());
        assertEquals(1, loads.driveLimitRequests);
        assertEquals(1, loads.rollerLimitRequests);
        assertEquals(1, loads.extenderLimitRequests);
        assertEquals(1, loads.indexerLimitRequests);
    }

    @Test
    void restoresStartupLimitsWhenAControlledCurrentSignalIsInvalid() {
        FakePowerIO power = new FakePowerIO(12.0, 60.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = 60.0;
        PowerBudgetManager manager = createManager(
                power, loads, new ConstantBudgetSource(100.0), PowerBudgetManager.Mode.ACTIVE, 1_000.0, 0.0);

        manager.updateForTest(0.02, true);
        loads.driveCurrentValid = false;
        PowerBudgetManager.Snapshot invalid = advanceUntilStartupLimitsRestore(manager);

        assertFalse(invalid.inputsValid());
        assertEquals(PowerBudgetManager.LimitingReason.INVALID_INPUT, invalid.limitingReason());
        assertTrue(invalid.startupLimitsRestored());
        assertEquals(EnergyConstants.DRIVE_STARTUP_LIMIT_AMPS, loads.lastDriveLimit, EPSILON);
    }

    @Test
    void blocksActiveWhenControlledCurrentImplausiblyExceedsThePowerDistributionTotal() {
        FakePowerIO power = new FakePowerIO(12.0, 60.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = 60.0;
        PowerBudgetManager manager = createManager(
                power, loads, new ConstantBudgetSource(100.0), PowerBudgetManager.Mode.ACTIVE, 1_000.0, 0.0);

        manager.updateForTest(0.02, true);
        power.totalCurrentAmps = 10.0;
        PowerBudgetManager.Snapshot mismatch = advanceUntilStartupLimitsRestore(manager);

        assertFalse(mismatch.inputsValid());
        assertEquals(PowerBudgetManager.LimitingReason.CURRENT_MEASUREMENT_MISMATCH, mismatch.limitingReason());
        assertEquals(50.0, mismatch.measurementMismatchAmps(), EPSILON);
        assertTrue(mismatch.startupLimitsRestored());
        assertEquals(EnergyConstants.DRIVE_STARTUP_LIMIT_AMPS, loads.lastDriveLimit, EPSILON);
    }

    @Test
    void persistentInputFaultRequiresModeExitBeforeActiveCanResume() {
        FakePowerIO power = new FakePowerIO(12.0, 60.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = 60.0;
        AtomicReference<PowerBudgetManager.Mode> mode = new AtomicReference<>(PowerBudgetManager.Mode.ACTIVE);
        PowerBudgetManager manager = createManager(power, loads, new ConstantBudgetSource(100.0), mode, 1_000.0, 0.0);

        manager.updateForTest(0.02, true);
        power.totalCurrentAmps = Double.NaN;
        PowerBudgetManager.Snapshot faultTransition = advanceUntilStartupLimitsRestore(manager);
        assertEquals(PowerBudgetManager.Mode.OFF, faultTransition.effectiveMode());
        assertEquals(2, loads.driveLimitRequests);

        power.totalCurrentAmps = 60.0;
        PowerBudgetManager.Snapshot stillLatched = manager.updateForTest(0.02, true);
        assertEquals(PowerBudgetManager.Mode.OFF, stillLatched.effectiveMode());
        assertEquals(2, loads.driveLimitRequests);

        mode.set(PowerBudgetManager.Mode.SHADOW);
        manager.updateForTest(0.02, true);
        mode.set(PowerBudgetManager.Mode.ACTIVE);
        PowerBudgetManager.Snapshot rearmed = manager.updateForTest(0.02, true);
        assertEquals(PowerBudgetManager.Mode.ACTIVE, rearmed.effectiveMode());
        assertEquals(3, loads.driveLimitRequests);
    }

    @Test
    void persistentModelFailureRestoresStartupLimitsAndLatchesActiveOff() {
        FakePowerIO power = new FakePowerIO(12.0, 60.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = 60.0;
        AtomicReference<PowerBudgetManager.Mode> mode = new AtomicReference<>(PowerBudgetManager.Mode.ACTIVE);
        ConstantBudgetSource budget = new ConstantBudgetSource(100.0);
        PowerBudgetManager manager = createManager(power, loads, budget, mode, 1_000.0, 0.0);

        manager.updateForTest(0.02, true);
        budget.throwOnUpdate = true;
        PowerBudgetManager.Snapshot faultTransition = advanceUntilStartupLimitsRestore(manager);

        assertFalse(faultTransition.inputsValid());
        assertEquals(PowerBudgetManager.LimitingReason.INVALID_INPUT, faultTransition.limitingReason());
        assertEquals(PowerBudgetManager.Mode.OFF, faultTransition.effectiveMode());
        assertEquals(EnergyConstants.DRIVE_STARTUP_LIMIT_AMPS, loads.lastDriveLimit, EPSILON);

        budget.throwOnUpdate = false;
        PowerBudgetManager.Snapshot stillLatched = manager.updateForTest(0.02, true);
        assertEquals(PowerBudgetManager.Mode.OFF, stillLatched.effectiveMode());
        assertEquals(2, loads.driveLimitRequests);
    }

    @Test
    void aBrownoutDropsImmediatelyAndRecoveryRisesAtConfiguredRate() {
        FakePowerIO power = new FakePowerIO(12.0, 120.0, false);
        FakeControlledLoads loads = new FakeControlledLoads();
        loads.driveCurrent = 120.0;
        ConstantBudgetSource budget = new ConstantBudgetSource(120.0);
        PowerBudgetManager manager = createManager(power, loads, budget, PowerBudgetManager.Mode.SHADOW, 50.0, 2.0);

        assertEquals(120.0, manager.updateForTest(0.02, true).filteredControlledPoolAmps(), EPSILON);
        budget.safeCurrentAmps = 70.0;
        power.brownedOut = true;
        assertEquals(70.0, manager.updateForTest(0.02, true).filteredControlledPoolAmps(), EPSILON);
        budget.safeCurrentAmps = 120.0;
        power.brownedOut = false;
        assertEquals(71.0, manager.updateForTest(0.02, true).filteredControlledPoolAmps(), EPSILON);
    }

    @Test
    void constructorsHaveNoShooterOrUpgoerDependencies() {
        boolean forbiddenDependency = Arrays.stream(PowerBudgetManager.class.getDeclaredConstructors())
                .flatMap(constructor -> Arrays.stream(constructor.getParameterTypes()))
                .map(Class::getName)
                .map(String::toLowerCase)
                .anyMatch(name -> name.contains("shooter") || name.contains("upgoer"));

        assertFalse(forbiddenDependency);
    }

    @Test
    void activeWritesAreAllowedOnlyWhenTheRobotIsTeleopEnabled() {
        assertEquals(
                PowerBudgetManager.Mode.OFF,
                PowerBudgetManager.resolveEffectiveMode(PowerBudgetManager.Mode.ACTIVE, false));
        assertEquals(
                PowerBudgetManager.Mode.ACTIVE,
                PowerBudgetManager.resolveEffectiveMode(PowerBudgetManager.Mode.ACTIVE, true));
        assertEquals(
                PowerBudgetManager.Mode.SHADOW,
                PowerBudgetManager.resolveEffectiveMode(PowerBudgetManager.Mode.SHADOW, true));
    }

    private static PowerBudgetManager createManager(
            FakePowerIO power,
            FakeControlledLoads loads,
            PowerBudgetManager.BudgetSource budget,
            PowerBudgetManager.Mode mode,
            double recoveryRate,
            double recoveryWindow) {
        return createManager(power, loads, budget, new AtomicReference<>(mode), recoveryRate, recoveryWindow);
    }

    private static PowerBudgetManager createManager(
            FakePowerIO power,
            FakeControlledLoads loads,
            PowerBudgetManager.BudgetSource budget,
            AtomicReference<PowerBudgetManager.Mode> mode,
            double recoveryRate,
            double recoveryWindow) {
        return new PowerBudgetManager(
                power,
                loads,
                budget,
                new StateAwareCurrentAllocator(ALLOCATION_CONFIG),
                new BrownoutRecoveryLimiter(recoveryRate, recoveryWindow),
                mode::get,
                false);
    }

    private static PowerBudgetManager.Snapshot advanceUntilStartupLimitsRestore(PowerBudgetManager manager) {
        for (int i = 0; i < 10; i++) {
            PowerBudgetManager.Snapshot snapshot = manager.updateForTest(0.02, true);
            if (snapshot.startupLimitsRestored()) {
                return snapshot;
            }
        }
        throw new AssertionError("startup limits were not restored after persistent invalid input");
    }

    private static final class FakePowerIO implements PowerIO {
        private double batteryVoltage;
        private double totalCurrentAmps;
        private boolean brownedOut;

        FakePowerIO(double batteryVoltage, double totalCurrentAmps, boolean brownedOut) {
            this.batteryVoltage = batteryVoltage;
            this.totalCurrentAmps = totalCurrentAmps;
            this.brownedOut = brownedOut;
        }

        @Override
        public void updateInputs(PowerIOInputs inputs) {
            inputs.batteryVoltage = batteryVoltage;
            inputs.totalCurrentAmps = totalCurrentAmps;
            inputs.brownedOut = brownedOut;
            inputs.connected = true;
        }
    }

    private static final class ConstantBudgetSource implements PowerBudgetManager.BudgetSource {
        private double safeCurrentAmps;
        private boolean throwOnUpdate;

        ConstantBudgetSource(double safeCurrentAmps) {
            this.safeCurrentAmps = safeCurrentAmps;
        }

        @Override
        public void reset(double batteryVoltage, double totalCurrentAmps) {}

        @Override
        public ModelBudget update(double totalCurrentAmps, double batteryVoltage, double dtSeconds) {
            if (throwOnUpdate) {
                throw new IllegalStateException("synthetic model failure");
            }
            return new ModelBudget(safeCurrentAmps, safeCurrentAmps, safeCurrentAmps);
        }
    }

    private static final class FakeControlledLoads implements PowerBudgetManager.ControlledLoads {
        private double driveCurrent;
        private double rollerCurrent;
        private double extenderCurrent;
        private double indexerCurrent;
        private boolean driveCurrentValid = true;
        private Activity activity = new Activity(false, false, false);
        private int driveLimitRequests;
        private int rollerLimitRequests;
        private int extenderLimitRequests;
        private int indexerLimitRequests;
        private double lastDriveLimit;
        private double lastRollerLimit;
        private double lastExtenderLimit;
        private double lastIndexerLimit;

        @Override
        public double driveSupplyCurrentAmps() {
            return driveCurrent;
        }

        @Override
        public double rollerSupplyCurrentAmps() {
            return rollerCurrent;
        }

        @Override
        public double extenderSupplyCurrentAmps() {
            return extenderCurrent;
        }

        @Override
        public double indexerSupplyCurrentAmps() {
            return indexerCurrent;
        }

        @Override
        public boolean driveSupplyCurrentValid() {
            return driveCurrentValid;
        }

        @Override
        public boolean rollerSupplyCurrentValid() {
            return true;
        }

        @Override
        public boolean extenderSupplyCurrentValid() {
            return true;
        }

        @Override
        public boolean indexerSupplyCurrentValid() {
            return true;
        }

        @Override
        public Activity activity() {
            return activity;
        }

        @Override
        public void requestDriveLimit(double perMotorAmps) {
            driveLimitRequests++;
            lastDriveLimit = perMotorAmps;
        }

        @Override
        public void requestRollerLimit(double perMotorAmps) {
            rollerLimitRequests++;
            lastRollerLimit = perMotorAmps;
        }

        @Override
        public void requestExtenderLimit(double amps) {
            extenderLimitRequests++;
            lastExtenderLimit = amps;
        }

        @Override
        public void requestIndexerLimit(double amps) {
            indexerLimitRequests++;
            lastIndexerLimit = amps;
        }

        int totalLimitRequestCount() {
            return driveLimitRequests + rollerLimitRequests + extenderLimitRequests + indexerLimitRequests;
        }
    }
}
