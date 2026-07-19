package frc.robot.energy;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.energy.BudgetMath.Result;
import frc.robot.energy.StateAwareCurrentAllocator.Activity;
import frc.robot.energy.StateAwareCurrentAllocator.Allocation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import java.util.Locale;
import java.util.Objects;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * Calculates and optionally applies supply-current limits to drive propulsion, intake rollers, the intake extender, and
 * the indexer. Shooter and upgoer types are intentionally absent from this class.
 */
public final class PowerBudgetManager {
    public enum Mode {
        OFF,
        SHADOW,
        ACTIVE;

        public static Mode parse(String value) {
            if (value == null) {
                return SHADOW;
            }
            try {
                return valueOf(value.trim().toUpperCase(Locale.ROOT));
            } catch (IllegalArgumentException exception) {
                return SHADOW;
            }
        }
    }

    public enum LimitingReason {
        NONE,
        BATTERY,
        BREAKER,
        ABSOLUTE_CEILING,
        ACTIVE_MINIMUM_DEFICIT,
        STANDBY_DEFICIT,
        CURRENT_MEASUREMENT_MISMATCH,
        INVALID_INPUT
    }

    public record ModelBudget(
            double batteryMaxCurrentAmps,
            double breakerMaxCurrentAmps,
            double safeRobotCurrentAmps,
            double batteryStateOfCharge,
            double batteryPolarizationVoltage,
            double breakerDamage) {
        public ModelBudget(double batteryMaxCurrentAmps, double breakerMaxCurrentAmps, double safeRobotCurrentAmps) {
            this(
                    batteryMaxCurrentAmps,
                    breakerMaxCurrentAmps,
                    safeRobotCurrentAmps,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN);
        }

        public ModelBudget {
            requireFiniteNonnegative(batteryMaxCurrentAmps, "batteryMaxCurrentAmps");
            requireFiniteNonnegative(breakerMaxCurrentAmps, "breakerMaxCurrentAmps");
            requireFiniteNonnegative(safeRobotCurrentAmps, "safeRobotCurrentAmps");
            requireFiniteOrNaN(batteryStateOfCharge, "batteryStateOfCharge");
            requireFiniteOrNaN(batteryPolarizationVoltage, "batteryPolarizationVoltage");
            requireFiniteOrNaN(breakerDamage, "breakerDamage");
        }
    }

    public record Snapshot(
            Mode requestedMode,
            Mode effectiveMode,
            boolean inputsValid,
            LimitingReason limitingReason,
            double batteryVoltage,
            double totalCurrentAmps,
            double controlledCurrentAmps,
            double uncontrolledCurrentAmps,
            double measurementMismatchAmps,
            ModelBudget modelBudget,
            double rawControlledPoolAmps,
            double filteredControlledPoolAmps,
            boolean recovering,
            Activity activity,
            Allocation allocation,
            boolean startupLimitsRestored) {}

    interface BudgetSource {
        void reset(double batteryVoltage, double totalCurrentAmps);

        ModelBudget update(double totalCurrentAmps, double batteryVoltage, double dtSeconds);
    }

    /**
     * A deliberately named, closed set of controlled loads. There is no generic registry where a shooter or upgoer can
     * be added accidentally.
     */
    interface ControlledLoads {
        double driveSupplyCurrentAmps();

        double rollerSupplyCurrentAmps();

        double extenderSupplyCurrentAmps();

        double indexerSupplyCurrentAmps();

        boolean driveSupplyCurrentValid();

        boolean rollerSupplyCurrentValid();

        boolean extenderSupplyCurrentValid();

        boolean indexerSupplyCurrentValid();

        Activity activity();

        void requestDriveLimit(double perMotorAmps);

        void requestRollerLimit(double perMotorAmps);

        void requestExtenderLimit(double amps);

        void requestIndexerLimit(double amps);
    }

    private static final Allocation EMPTY_ALLOCATION = new Allocation(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);
    private static final ModelBudget EMPTY_MODEL_BUDGET = new ModelBudget(0.0, 0.0, 0.0);

    private final PowerIO powerIO;
    private final PowerIOInputsAutoLogged powerInputs = new PowerIOInputsAutoLogged();
    private final ControlledLoads controlledLoads;
    private final BudgetSource budgetSource;
    private final StateAwareCurrentAllocator allocator;
    private final BrownoutRecoveryLimiter recoveryLimiter;
    private final Supplier<Mode> requestedModeSupplier;
    private final boolean loggingEnabled;
    private final Alert invalidInputsAlert;
    private final Alert measurementMismatchAlert;
    private final Alert allocationDeficitAlert;

    private boolean modelInitialized;
    private boolean recoveryInitialized;
    private boolean dynamicLimitsApplied;
    private boolean activeFaultLatched;
    private double invalidActiveInputDurationSeconds;
    private double lastUpdateTimestampSeconds = Double.NaN;
    private Snapshot snapshot = new Snapshot(
            Mode.SHADOW,
            Mode.OFF,
            false,
            LimitingReason.INVALID_INPUT,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            EMPTY_MODEL_BUDGET,
            0.0,
            0.0,
            false,
            new Activity(false, false, false),
            EMPTY_ALLOCATION,
            false);

    public PowerBudgetManager(PowerIO powerIO, Drive drive, Intake intake, Indexer indexer) {
        this(
                powerIO,
                new RobotControlledLoads(drive, intake, indexer),
                new ModelBudgetSource(),
                new StateAwareCurrentAllocator(EnergyConstants.initialAllocationConfig()),
                new BrownoutRecoveryLimiter(
                        EnergyConstants.BROWNOUT_RECOVERY_RATE_AMPS_PER_SECOND,
                        EnergyConstants.BROWNOUT_RECOVERY_WINDOW_SECONDS),
                new DashboardModeSource(),
                true);
    }

    PowerBudgetManager(
            PowerIO powerIO,
            ControlledLoads controlledLoads,
            BudgetSource budgetSource,
            StateAwareCurrentAllocator allocator,
            BrownoutRecoveryLimiter recoveryLimiter,
            Supplier<Mode> requestedModeSupplier,
            boolean loggingEnabled) {
        this.powerIO = Objects.requireNonNull(powerIO);
        this.controlledLoads = Objects.requireNonNull(controlledLoads);
        this.budgetSource = Objects.requireNonNull(budgetSource);
        this.allocator = Objects.requireNonNull(allocator);
        this.recoveryLimiter = Objects.requireNonNull(recoveryLimiter);
        this.requestedModeSupplier = Objects.requireNonNull(requestedModeSupplier);
        this.loggingEnabled = loggingEnabled;
        invalidInputsAlert = loggingEnabled
                ? new Alert("Power management received invalid or disconnected power data.", AlertType.kWarning)
                : null;
        measurementMismatchAlert = loggingEnabled
                ? new Alert(
                        "Controlled Talon current implausibly exceeds the power-distribution total.",
                        AlertType.kWarning)
                : null;
        allocationDeficitAlert = loggingEnabled
                ? new Alert("Power budget cannot satisfy configured controlled-load minimums.", AlertType.kWarning)
                : null;
    }

    /** Runs after the command scheduler so mechanism activity reflects the current scheduler cycle. */
    public void update() {
        double nowSeconds = Timer.getFPGATimestamp();
        double dtSeconds = Double.isFinite(lastUpdateTimestampSeconds)
                ? nowSeconds - lastUpdateTimestampSeconds
                : EnergyConstants.LOOP_PERIOD_SECONDS;
        lastUpdateTimestampSeconds = nowSeconds;
        if (!Double.isFinite(dtSeconds) || dtSeconds <= 0.0 || dtSeconds > 0.25) {
            dtSeconds = EnergyConstants.LOOP_PERIOD_SECONDS;
        }
        updateInternal(dtSeconds, DriverStation.isTeleopEnabled(), true);
    }

    Snapshot updateForTest(double dtSeconds, boolean activeWritesAllowed) {
        return updateInternal(dtSeconds, activeWritesAllowed, false);
    }

    /**
     * Clears model initialization and restores the configured startup limits if dynamic limits had been active. Breaker
     * damage intentionally persists because disabling the robot does not cool the breaker instantly.
     */
    public void reset() {
        if (dynamicLimitsApplied) {
            restoreStartupLimits();
            dynamicLimitsApplied = false;
        }
        modelInitialized = false;
        recoveryInitialized = false;
        activeFaultLatched = false;
        invalidActiveInputDurationSeconds = 0.0;
        lastUpdateTimestampSeconds = Double.NaN;
    }

    public Snapshot getSnapshot() {
        return snapshot;
    }

    private Snapshot updateInternal(double dtSeconds, boolean activeWritesAllowed, boolean log) {
        requireFinitePositive(dtSeconds, "dtSeconds");
        powerIO.updateInputs(powerInputs);
        if (log && loggingEnabled) {
            Logger.processInputs("PowerManagement/PowerIO", powerInputs);
        }

        Mode requestedMode = Objects.requireNonNullElse(requestedModeSupplier.get(), Mode.SHADOW);
        if (requestedMode != Mode.ACTIVE) {
            activeFaultLatched = false;
            invalidActiveInputDurationSeconds = 0.0;
        }
        Mode effectiveMode = activeFaultLatched ? Mode.OFF : resolveEffectiveMode(requestedMode, activeWritesAllowed);
        Activity activity = safeActivity();

        double controlledCurrentAmps = sumControlledCurrent();
        boolean inputsValid = powerInputs.connected
                && Double.isFinite(powerInputs.batteryVoltage)
                && powerInputs.batteryVoltage > 0.0
                && Double.isFinite(powerInputs.totalCurrentAmps)
                && powerInputs.totalCurrentAmps >= 0.0
                && Double.isFinite(controlledCurrentAmps)
                && controlledCurrentAmps >= 0.0
                && controlledCurrentsValid();

        if (!inputsValid) {
            InvalidHandling invalidHandling = handleInvalidInput(effectiveMode, dtSeconds);
            effectiveMode = invalidHandling.effectiveMode();
            snapshot = new Snapshot(
                    requestedMode,
                    effectiveMode,
                    false,
                    LimitingReason.INVALID_INPUT,
                    powerInputs.batteryVoltage,
                    powerInputs.totalCurrentAmps,
                    controlledCurrentAmps,
                    0.0,
                    0.0,
                    EMPTY_MODEL_BUDGET,
                    0.0,
                    0.0,
                    false,
                    activity,
                    EMPTY_ALLOCATION,
                    invalidHandling.startupLimitsRestored());
            publish(snapshot, log);
            return snapshot;
        }

        double measurementMismatchAmps = Math.max(0.0, controlledCurrentAmps - powerInputs.totalCurrentAmps);
        if (measurementMismatchAmps > EnergyConstants.MAXIMUM_CONTROLLED_CURRENT_MISMATCH_AMPS) {
            InvalidHandling invalidHandling = handleInvalidInput(effectiveMode, dtSeconds);
            effectiveMode = invalidHandling.effectiveMode();
            snapshot = new Snapshot(
                    requestedMode,
                    effectiveMode,
                    false,
                    LimitingReason.CURRENT_MEASUREMENT_MISMATCH,
                    powerInputs.batteryVoltage,
                    powerInputs.totalCurrentAmps,
                    controlledCurrentAmps,
                    0.0,
                    measurementMismatchAmps,
                    EMPTY_MODEL_BUDGET,
                    0.0,
                    0.0,
                    false,
                    activity,
                    EMPTY_ALLOCATION,
                    invalidHandling.startupLimitsRestored());
            publish(snapshot, log);
            return snapshot;
        }

        ModelBudget modelBudget;
        try {
            if (!modelInitialized) {
                budgetSource.reset(powerInputs.batteryVoltage, powerInputs.totalCurrentAmps);
                modelInitialized = true;
            }
            modelBudget = budgetSource.update(powerInputs.totalCurrentAmps, powerInputs.batteryVoltage, dtSeconds);
        } catch (RuntimeException exception) {
            InvalidHandling invalidHandling = handleInvalidInput(effectiveMode, dtSeconds);
            effectiveMode = invalidHandling.effectiveMode();
            snapshot = new Snapshot(
                    requestedMode,
                    effectiveMode,
                    false,
                    LimitingReason.INVALID_INPUT,
                    powerInputs.batteryVoltage,
                    powerInputs.totalCurrentAmps,
                    controlledCurrentAmps,
                    0.0,
                    0.0,
                    EMPTY_MODEL_BUDGET,
                    0.0,
                    0.0,
                    false,
                    activity,
                    EMPTY_ALLOCATION,
                    invalidHandling.startupLimitsRestored());
            if (log && loggingEnabled) {
                Logger.recordOutput("PowerManagement/ModelError", exception.toString());
            }
            publish(snapshot, log);
            return snapshot;
        }
        invalidActiveInputDurationSeconds = 0.0;

        Result budgetMath = BudgetMath.derive(
                modelBudget.safeRobotCurrentAmps(), powerInputs.totalCurrentAmps, controlledCurrentAmps);
        double rawControlledPoolAmps = budgetMath.controlledPoolAmps();
        if (!recoveryInitialized) {
            recoveryLimiter.reset(rawControlledPoolAmps);
            recoveryInitialized = true;
        }
        double filteredControlledPoolAmps =
                recoveryLimiter.update(rawControlledPoolAmps, powerInputs.brownedOut, dtSeconds);
        Allocation allocation = allocator.allocate(filteredControlledPoolAmps, activity);

        LimitingReason limitingReason = determineLimitingReason(modelBudget, allocation);
        boolean restored = false;
        if (effectiveMode == Mode.ACTIVE) {
            applyAllocation(allocation);
            dynamicLimitsApplied = true;
        } else if (dynamicLimitsApplied) {
            restoreStartupLimits();
            dynamicLimitsApplied = false;
            restored = true;
        }

        snapshot = new Snapshot(
                requestedMode,
                effectiveMode,
                true,
                limitingReason,
                powerInputs.batteryVoltage,
                powerInputs.totalCurrentAmps,
                controlledCurrentAmps,
                budgetMath.uncontrolledCurrentAmps(),
                budgetMath.measurementMismatchAmps(),
                modelBudget,
                rawControlledPoolAmps,
                filteredControlledPoolAmps,
                recoveryLimiter.isRecovering(),
                activity,
                allocation,
                restored);
        publish(snapshot, log);
        return snapshot;
    }

    private Activity safeActivity() {
        Activity activity = controlledLoads.activity();
        return activity != null ? activity : new Activity(false, false, false);
    }

    private double sumControlledCurrent() {
        return nonnegativeCurrent(controlledLoads.driveSupplyCurrentAmps())
                + nonnegativeCurrent(controlledLoads.rollerSupplyCurrentAmps())
                + nonnegativeCurrent(controlledLoads.extenderSupplyCurrentAmps())
                + nonnegativeCurrent(controlledLoads.indexerSupplyCurrentAmps());
    }

    private boolean controlledCurrentsValid() {
        return controlledLoads.driveSupplyCurrentValid()
                && controlledLoads.rollerSupplyCurrentValid()
                && controlledLoads.extenderSupplyCurrentValid()
                && controlledLoads.indexerSupplyCurrentValid();
    }

    private static double nonnegativeCurrent(double currentAmps) {
        return Math.max(0.0, currentAmps);
    }

    static Mode resolveEffectiveMode(Mode requestedMode, boolean activeWritesAllowed) {
        return activeWritesAllowed ? Objects.requireNonNull(requestedMode) : Mode.OFF;
    }

    private boolean restoreAfterDynamicFailure() {
        if (!dynamicLimitsApplied) {
            return false;
        }
        restoreStartupLimits();
        dynamicLimitsApplied = false;
        return true;
    }

    private InvalidHandling handleInvalidInput(Mode effectiveMode, double dtSeconds) {
        if (effectiveMode == Mode.ACTIVE) {
            invalidActiveInputDurationSeconds += dtSeconds;
            if (invalidActiveInputDurationSeconds < EnergyConstants.INVALID_ACTIVE_INPUT_GRACE_SECONDS) {
                return new InvalidHandling(effectiveMode, false);
            }
            activeFaultLatched = true;
            effectiveMode = Mode.OFF;
        } else {
            invalidActiveInputDurationSeconds = 0.0;
        }
        return new InvalidHandling(effectiveMode, restoreAfterDynamicFailure());
    }

    private void applyAllocation(Allocation allocation) {
        controlledLoads.requestDriveLimit(allocation.drivePerMotor());
        controlledLoads.requestRollerLimit(allocation.rollersPerMotor());
        controlledLoads.requestExtenderLimit(allocation.extenderPerMotor());
        controlledLoads.requestIndexerLimit(allocation.indexerPerMotor());
    }

    private void restoreStartupLimits() {
        controlledLoads.requestDriveLimit(EnergyConstants.DRIVE_STARTUP_LIMIT_AMPS);
        controlledLoads.requestRollerLimit(EnergyConstants.ROLLER_STARTUP_LIMIT_AMPS);
        controlledLoads.requestExtenderLimit(EnergyConstants.EXTENDER_STARTUP_LIMIT_AMPS);
        controlledLoads.requestIndexerLimit(EnergyConstants.INDEXER_STARTUP_LIMIT_AMPS);
    }

    private static LimitingReason determineLimitingReason(ModelBudget modelBudget, Allocation allocation) {
        if (allocation.activeMinimumDeficitAmps() > 0.0) {
            return LimitingReason.ACTIVE_MINIMUM_DEFICIT;
        }
        if (allocation.standbyDeficit()) {
            return LimitingReason.STANDBY_DEFICIT;
        }
        if (modelBudget.safeRobotCurrentAmps() >= EnergyConstants.ABSOLUTE_ROBOT_CURRENT_CEILING_AMPS - 1e-9) {
            return LimitingReason.ABSOLUTE_CEILING;
        }
        return modelBudget.batteryMaxCurrentAmps() <= modelBudget.breakerMaxCurrentAmps()
                ? LimitingReason.BATTERY
                : LimitingReason.BREAKER;
    }

    private void publish(Snapshot value, boolean log) {
        if (!log || !loggingEnabled) {
            return;
        }
        Logger.recordOutput("PowerManagement/ModelSource", EnergyConstants.MODEL_SOURCE);
        Logger.recordOutput(
                "PowerManagement/RequestedMode", value.requestedMode().name());
        Logger.recordOutput(
                "PowerManagement/EffectiveMode", value.effectiveMode().name());
        Logger.recordOutput("PowerManagement/InputsValid", value.inputsValid());
        Logger.recordOutput(
                "PowerManagement/LimitingReason", value.limitingReason().name());
        Logger.recordOutput("PowerManagement/Measured/TotalCurrentAmps", value.totalCurrentAmps());
        Logger.recordOutput("PowerManagement/Measured/ControlledCurrentAmps", value.controlledCurrentAmps());
        Logger.recordOutput("PowerManagement/Measured/UncontrolledCurrentAmps", value.uncontrolledCurrentAmps());
        Logger.recordOutput("PowerManagement/Measured/MismatchAmps", value.measurementMismatchAmps());
        Logger.recordOutput(
                "PowerManagement/Model/BatteryMaxCurrentAmps",
                value.modelBudget().batteryMaxCurrentAmps());
        Logger.recordOutput(
                "PowerManagement/Model/BreakerMaxCurrentAmps",
                value.modelBudget().breakerMaxCurrentAmps());
        Logger.recordOutput(
                "PowerManagement/Model/SafeRobotCurrentAmps",
                value.modelBudget().safeRobotCurrentAmps());
        Logger.recordOutput(
                "PowerManagement/Model/BatteryStateOfCharge",
                value.modelBudget().batteryStateOfCharge());
        Logger.recordOutput(
                "PowerManagement/Model/BatteryPolarizationVoltage",
                value.modelBudget().batteryPolarizationVoltage());
        Logger.recordOutput(
                "PowerManagement/Model/BreakerDamage", value.modelBudget().breakerDamage());
        Logger.recordOutput("PowerManagement/Pool/RawAmps", value.rawControlledPoolAmps());
        Logger.recordOutput("PowerManagement/Pool/FilteredAmps", value.filteredControlledPoolAmps());
        Logger.recordOutput("PowerManagement/Pool/Recovering", value.recovering());
        Logger.recordOutput("PowerManagement/Activity/Rollers", value.activity().rollersActive());
        Logger.recordOutput(
                "PowerManagement/Activity/RollersIdle", value.activity().rollersIdle());
        Logger.recordOutput(
                "PowerManagement/Activity/Extender", value.activity().extenderActive());
        Logger.recordOutput("PowerManagement/Activity/Indexer", value.activity().indexerActive());
        Logger.recordOutput(
                "PowerManagement/Allocation/DrivePerMotorAmps",
                value.allocation().drivePerMotor());
        Logger.recordOutput(
                "PowerManagement/Allocation/RollersPerMotorAmps",
                value.allocation().rollersPerMotor());
        Logger.recordOutput(
                "PowerManagement/Allocation/ExtenderAmps", value.allocation().extenderPerMotor());
        Logger.recordOutput(
                "PowerManagement/Allocation/IndexerAmps", value.allocation().indexerPerMotor());
        Logger.recordOutput(
                "PowerManagement/Allocation/AllocatedAmps", value.allocation().allocatedAmps());
        Logger.recordOutput(
                "PowerManagement/Allocation/UnusedAmps", value.allocation().unusedAmps());
        Logger.recordOutput(
                "PowerManagement/Allocation/ActiveMinimumDeficitAmps",
                value.allocation().activeMinimumDeficitAmps());
        Logger.recordOutput(
                "PowerManagement/Allocation/IrreducibleFloorDeficitAmps",
                value.allocation().irreducibleFloorDeficitAmps());
        Logger.recordOutput(
                "PowerManagement/Allocation/StandbyDeficit", value.allocation().standbyDeficit());
        Logger.recordOutput("PowerManagement/StartupLimitsRestored", value.startupLimitsRestored());
        Logger.recordOutput("PowerManagement/ActiveFaultLatched", activeFaultLatched);
        Logger.recordOutput("PowerManagement/InvalidActiveInputDurationSeconds", invalidActiveInputDurationSeconds);
        invalidInputsAlert.set(!value.inputsValid());
        measurementMismatchAlert.set(value.limitingReason() == LimitingReason.CURRENT_MEASUREMENT_MISMATCH);
        allocationDeficitAlert.set(value.allocation().standbyDeficit()
                || value.allocation().activeMinimumDeficitAmps() > 0.0
                || value.allocation().irreducibleFloorDeficitAmps() > 0.0);
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

    private static void requireFiniteOrNaN(double value, String name) {
        if (!Double.isFinite(value) && !Double.isNaN(value)) {
            throw new IllegalArgumentException(name + " must be finite or NaN");
        }
    }

    private static final class DashboardModeSource implements Supplier<Mode> {
        private final LoggedNetworkString mode = new LoggedNetworkString("PowerManagement/Mode", Mode.SHADOW.name());

        @Override
        public Mode get() {
            return Mode.parse(mode.get());
        }
    }

    private record InvalidHandling(Mode effectiveMode, boolean startupLimitsRestored) {}

    private static final class ModelBudgetSource implements BudgetSource {
        private final BatteryEstimator battery =
                new BatteryEstimator(BatteryEstimator.Parameters.mechanicalAdvantageMkPoweredReference());
        private final BreakerModel breaker = new BreakerModel(EnergyConstants.BREAKER_NICENESS);

        @Override
        public void reset(double batteryVoltage, double totalCurrentAmps) {
            battery.reset(batteryVoltage, totalCurrentAmps);
        }

        @Override
        public ModelBudget update(double totalCurrentAmps, double batteryVoltage, double dtSeconds) {
            battery.update(totalCurrentAmps, batteryVoltage, dtSeconds);
            breaker.update(totalCurrentAmps, dtSeconds);
            double batteryMaxCurrentAmps = battery.maxCurrentAtVoltage(EnergyConstants.MINIMUM_BATTERY_VOLTAGE);
            double breakerMaxCurrentAmps = breaker.maxCurrentFor(EnergyConstants.BREAKER_DANGER_HORIZON_SECONDS);
            double safeRobotCurrentAmps = Math.min(
                    EnergyConstants.ABSOLUTE_ROBOT_CURRENT_CEILING_AMPS,
                    EnergyConstants.BUDGET_HEADROOM * Math.min(batteryMaxCurrentAmps, breakerMaxCurrentAmps));
            return new ModelBudget(
                    batteryMaxCurrentAmps,
                    breakerMaxCurrentAmps,
                    safeRobotCurrentAmps,
                    battery.stateOfCharge(),
                    battery.polarizationVoltage(),
                    breaker.damage());
        }
    }

    private static final class RobotControlledLoads implements ControlledLoads {
        private final Drive drive;
        private final Intake intake;
        private final Indexer indexer;

        RobotControlledLoads(Drive drive, Intake intake, Indexer indexer) {
            this.drive = Objects.requireNonNull(drive);
            this.intake = Objects.requireNonNull(intake);
            this.indexer = Objects.requireNonNull(indexer);
        }

        @Override
        public double driveSupplyCurrentAmps() {
            return drive.getDriveSupplyCurrentAmps();
        }

        @Override
        public double rollerSupplyCurrentAmps() {
            return intake.getRollerSupplyCurrentAmps();
        }

        @Override
        public double extenderSupplyCurrentAmps() {
            return intake.getExtenderSupplyCurrentAmps();
        }

        @Override
        public double indexerSupplyCurrentAmps() {
            return indexer.getSupplyCurrentAmps();
        }

        @Override
        public boolean driveSupplyCurrentValid() {
            return drive.isDriveSupplyCurrentValid();
        }

        @Override
        public boolean rollerSupplyCurrentValid() {
            return intake.isRollerSupplyCurrentValid();
        }

        @Override
        public boolean extenderSupplyCurrentValid() {
            return intake.isExtenderSupplyCurrentValid();
        }

        @Override
        public boolean indexerSupplyCurrentValid() {
            return indexer.isSupplyCurrentValid();
        }

        @Override
        public Activity activity() {
            return new Activity(
                    intake.getRollerPowerManagementActivity(),
                    intake.isExtenderActiveForPowerManagement(),
                    indexer.isPowerManagementActive());
        }

        @Override
        public void requestDriveLimit(double perMotorAmps) {
            drive.setDriveSupplyCurrentLimit(perMotorAmps);
        }

        @Override
        public void requestRollerLimit(double perMotorAmps) {
            intake.setRollerSupplyCurrentLimit(perMotorAmps);
        }

        @Override
        public void requestExtenderLimit(double amps) {
            intake.setExtenderSupplyCurrentLimit(amps);
        }

        @Override
        public void requestIndexerLimit(double amps) {
            indexer.setSupplyCurrentLimit(amps);
        }
    }
}
