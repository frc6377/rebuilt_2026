// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.energy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class FinanceDepartment extends FullSubsystem {
    // MARK: - Constants
    private static final double minVoltageBrownout = 7.0;
    private static final double maxBudgetAmps = 200.0;
    private static final double breakerNiceness = 0.05;
    private static final double budgetWarningThreshold = 180.0;
    private static final double budgetHeadroom = 0.9;
    private static final double breakerDangerHorizonSecs = 3.0;
    private final double breakerDamageWarningThreshold;

    private static final Alert budgetWarning =
            new Alert("Battery is low, robot performance may be degraded.", AlertType.kInfo);
    private static final Debouncer budgetWarningDebouncer = new Debouncer(0.5, DebounceType.kBoth);
    private static final Alert brownoutWarning =
            new Alert("Brownout detected, controlled-load performance may be degraded.", AlertType.kWarning);
    private static final Alert breakerDamageWarning =
            new Alert("Breaker damage is high, please stop using the robot.", AlertType.kWarning);
    private static final Debouncer breakerDamageWarningDebouncer = new Debouncer(0.5, DebounceType.kBoth);
    private static final Alert budgetDeficitWarning =
            new Alert("Controlled current budget below active-safe floors.", AlertType.kWarning);
    private static final Alert budgetManagerFaultWarning =
            new Alert("Power manager fault — using static supply limits.", AlertType.kWarning);

    private static FinanceDepartment instance;

    public static FinanceDepartment getInstance() {
        if (instance == null) instance = new FinanceDepartment();
        return instance;
    }


    // MARK: - Members
    private final BatteryLogger energyLogger = new BatteryLogger();
    private final BatteryEstimator battery = new BatteryEstimator();
    private final BreakerModel breaker = new BreakerModel(breakerNiceness);
    private final PowerDistribution pdh = new PowerDistribution(Constants.CANIDs.kPdhCanId, ModuleType.kRev);
    private final BatteryIOInputsAutoLogged inputs = new BatteryIOInputsAutoLogged();
    private final LoggedNetworkBoolean dynamicLimiting =
            new LoggedNetworkBoolean("FinanceDepartment/DynamicLimiting", CurrentLimits.Shared.dynamicLimitingEnabled);

    private double budget = 0.0;
    private double controlledPool = 0.0;
    private double indexerGroupBudget = 0.0;
    private double extenderGroupBudget = 0.0;
    private double rollerGroupBudget = 0.0;
    private double driveGroupBudget = 0.0;

    private double indexerLimit = CurrentLimits.Static.indexerSupplyAmps;
    private double extenderLimit = CurrentLimits.Static.extenderSupplyAmps;
    private double rollerLimit = CurrentLimits.Static.rollerSupplyAmps;
    private double driveLimit = CurrentLimits.Static.driveSupplyAmps;

    private final Debouncer brownoutDebouncer = new Debouncer(2.0, DebounceType.kFalling);

    private MechanismStates.Drive driveState = MechanismStates.Drive.IDLE;
    private MechanismStates.Roller rollerState = MechanismStates.Roller.OFF;
    private MechanismStates.Extender extenderState = MechanismStates.Extender.IDLE;
    private MechanismStates.Indexer indexerState = MechanismStates.Indexer.OFF;
    private MechanismStates.Shooter shooterState = MechanismStates.Shooter.IDLE;
    private MechanismStates.Upgoer upgoerState = MechanismStates.Upgoer.OFF;

    private boolean shooterSpinningThisCycle = false;
    private boolean upgoerRunningThisCycle = false;

    private FinanceDepartment() {
        breakerDamageWarningThreshold = (1.0 - breakerNiceness)
                - (breakerDangerHorizonSecs / BreakerModel.getTripTime(maxBudgetAmps / BreakerModel.I_RATED));
    }

    public void reset() {
        battery.setInitialVoltage(inputs.batteryVoltage, energyLogger.getTotalCurrent());
    }

    public void reportCurrentUsage(String key, boolean controlled, double... amps) {
        double totalAmps = 0.0;
        for (double amp : amps) totalAmps += Math.max(0.0, amp);
        energyLogger.reportCurrentUsage(key, controlled, totalAmps);
    }

    public void reportState(MechanismStates.Drive state) {
        driveState = state;
    }

    public void reportState(MechanismStates.Roller state) {
        rollerState = state;
    }

    public void reportState(MechanismStates.Extender state) {
        extenderState = state;
    }

    public void reportState(MechanismStates.Indexer state) {
        indexerState = state;
    }

    public void reportState(MechanismStates.Shooter state) {
        if (state == MechanismStates.Shooter.SPINNING) {
            shooterSpinningThisCycle = true;
        }
    }

    public void reportState(MechanismStates.Upgoer state) {
        if (state == MechanismStates.Upgoer.RUNNING) {
            upgoerRunningThisCycle = true;
        }
    }

    @Override
    public void periodic() {
        inputs.batteryVoltage = pdh.getVoltage();
        inputs.rioCurrent = RobotController.getInputCurrent();
        inputs.brownedOut = RobotController.isBrownedOut();
        Logger.processInputs("EnergyLogger", inputs);
        energyLogger.setBatteryVoltage(inputs.batteryVoltage);
        energyLogger.setRioCurrent(inputs.rioCurrent);
    }

    @Override
    public void periodicAfterScheduler() {
        try {
            energyLogger.periodicAfterScheduler();

            double measuredTotal = pdh.getTotalCurrent();
            battery.update(measuredTotal, inputs.batteryVoltage);
            breaker.update(measuredTotal);

            double batteryMaxCurrent = battery.calculateMaxCurrent(minVoltageBrownout);
            double breakerMaxCurrent = breaker.calculateMaxCurrent(breakerDangerHorizonSecs);
            Logger.recordOutput("FinanceDepartment/BatteryMaxCurrent", batteryMaxCurrent);
            Logger.recordOutput("FinanceDepartment/BreakerMaxCurrent", breakerMaxCurrent);
            budget = Math.min(Math.min(batteryMaxCurrent, breakerMaxCurrent) * budgetHeadroom, maxBudgetAmps);

            double controlledMeasured = energyLogger.getControlledCurrent();
            double uncontrolledMeasured = Math.max(0.0, measuredTotal - controlledMeasured);

            boolean brownoutDebounced = brownoutDebouncer.calculate(inputs.brownedOut);
            double rawControlledPool = Math.max(0.0, budget - uncontrolledMeasured);
            if (!brownoutDebounced) {
                controlledPool = rawControlledPool;
            } else if (rawControlledPool < controlledPool) {
                controlledPool = rawControlledPool;
            } else {
                controlledPool = Math.min(
                        rawControlledPool,
                        controlledPool + CurrentLimits.Shared.recoveryRateAmpsPerSec * LoggedRobot.defaultPeriodSecs);
            }
            controlledPool = Math.max(0.0, controlledPool);

            shooterState = shooterSpinningThisCycle ? MechanismStates.Shooter.SPINNING : MechanismStates.Shooter.IDLE;
            upgoerState = upgoerRunningThisCycle ? MechanismStates.Upgoer.RUNNING : MechanismStates.Upgoer.OFF;
            shooterSpinningThisCycle = false;
            upgoerRunningThisCycle = false;

            allocate(controlledPool);
            updateAppliedLimits();

            Logger.recordOutput("FinanceDepartment/Budget", budget);
            Logger.recordOutput("FinanceDepartment/ControlledPool", controlledPool);
            Logger.recordOutput("FinanceDepartment/RawControlledPool", rawControlledPool);
            Logger.recordOutput("FinanceDepartment/PdhTotalCurrent", measuredTotal);
            Logger.recordOutput("FinanceDepartment/ControlledMeasured", controlledMeasured);
            Logger.recordOutput("FinanceDepartment/UncontrolledMeasured", uncontrolledMeasured);
            Logger.recordOutput("FinanceDepartment/DynamicLimiting", dynamicLimiting.get());
            Logger.recordOutput("FinanceDepartment/IndexerLimit", indexerLimit);
            Logger.recordOutput("FinanceDepartment/ExtenderLimit", extenderLimit);
            Logger.recordOutput("FinanceDepartment/RollerLimit", rollerLimit);
            Logger.recordOutput("FinanceDepartment/DriveLimit", driveLimit);
            Logger.recordOutput("FinanceDepartment/BudgetManagerFault", false);

            budgetWarning.set(budgetWarningDebouncer.calculate(budget < budgetWarningThreshold));
            brownoutWarning.set(brownoutDebounced);
            breakerDamageWarning.set(
                    breakerDamageWarningDebouncer.calculate(breaker.getDamageState() > breakerDamageWarningThreshold));
            budgetManagerFaultWarning.set(false);
        } catch (Exception e) {
            applyStaticLimits();
            budgetManagerFaultWarning.set(true);
            Logger.recordOutput("FinanceDepartment/BudgetManagerFault", true);
            Logger.recordOutput("FinanceDepartment/BudgetManagerFaultMessage", e.toString());
            Logger.recordOutput("FinanceDepartment/IndexerLimit", indexerLimit);
            Logger.recordOutput("FinanceDepartment/ExtenderLimit", extenderLimit);
            Logger.recordOutput("FinanceDepartment/RollerLimit", rollerLimit);
            Logger.recordOutput("FinanceDepartment/DriveLimit", driveLimit);
        } finally {
            energyLogger.resetTotals();
        }
    }

    private void allocate(double pool) {
        boolean indexerActive = indexerState == MechanismStates.Indexer.ACTIVE;
        boolean extenderActive = extenderState == MechanismStates.Extender.MOVING;
        boolean driveEnabled = driveState != MechanismStates.Drive.DISABLED;

        double indexerFloor = indexerActive ? CurrentLimits.Indexer.activeMinAmps : CurrentLimits.Indexer.standbyAmps;
        double extenderFloor =
                extenderActive ? CurrentLimits.Extender.activeMinAmps : CurrentLimits.Extender.standbyAmps;
        double rollerFloor =
                switch (rollerState) {
                    case OFF -> CurrentLimits.Roller.offMinAmps;
                    case IDLE -> CurrentLimits.Roller.idleMinAmps;
                    case ACTIVE -> CurrentLimits.Roller.activeMinAmps;
                };
        double driveFloor = driveEnabled ? CurrentLimits.Drive.standbyAmps : 0.0;

        double indexerTarget = indexerActive ? CurrentLimits.Indexer.targetAmps : indexerFloor;
        double extenderTarget = extenderActive ? CurrentLimits.Extender.targetAmps : extenderFloor;
        double rollerTarget =
                switch (rollerState) {
                    case OFF, IDLE -> rollerFloor;
                    case ACTIVE -> CurrentLimits.Roller.targetAmps;
                };
        double driveTarget = driveEnabled ? CurrentLimits.Drive.targetAmps : 0.0;

        double indexerMax = CurrentLimits.Indexer.maxAmps;
        double extenderMax = CurrentLimits.Extender.maxAmps;
        double rollerMax = CurrentLimits.Roller.maxAmps;
        double driveMax = CurrentLimits.Drive.maxLimitAmps * CurrentLimits.Drive.motorCount;

        double floors = indexerFloor + extenderFloor + rollerFloor + driveFloor;
        boolean deficit = pool < floors;
        budgetDeficitWarning.set(deficit);
        Logger.recordOutput("FinanceDepartment/BudgetDeficit", deficit);

        if (deficit) {
            double remaining = pool;
            indexerGroupBudget = Math.min(indexerFloor, remaining);
            remaining -= indexerGroupBudget;
            extenderGroupBudget = Math.min(extenderFloor, remaining);
            remaining -= extenderGroupBudget;
            rollerGroupBudget = Math.min(rollerFloor, remaining);
            remaining -= rollerGroupBudget;
            driveGroupBudget = Math.min(driveFloor, remaining);
            return;
        }

        indexerGroupBudget = indexerFloor;
        extenderGroupBudget = extenderFloor;
        rollerGroupBudget = rollerFloor;
        driveGroupBudget = driveFloor;
        double remaining = pool - floors;

        remaining = fillToward(
                remaining, indexerGroupBudget, Math.min(indexerTarget, indexerMax), v -> indexerGroupBudget = v);
        remaining = fillToward(
                remaining, extenderGroupBudget, Math.min(extenderTarget, extenderMax), v -> extenderGroupBudget = v);
        remaining =
                fillToward(remaining, rollerGroupBudget, Math.min(rollerTarget, rollerMax), v -> rollerGroupBudget = v);
        remaining = fillToward(remaining, driveGroupBudget, Math.min(driveTarget, driveMax), v -> driveGroupBudget = v);

        driveGroupBudget += Math.min(driveMax - driveGroupBudget, remaining);
    }

    private void updateAppliedLimits() {
        if (!dynamicLimiting.get()) {
            applyStaticLimits();
            return;
        }

        indexerLimit = quantize(indexerGroupBudget);
        extenderLimit = quantize(extenderGroupBudget);
        rollerLimit = quantize(rollerGroupBudget / CurrentLimits.Roller.motorCount);
        driveLimit = quantize(MathUtil.clamp(
                driveGroupBudget / CurrentLimits.Drive.motorCount,
                CurrentLimits.Drive.minLimitAmps,
                CurrentLimits.Drive.maxLimitAmps));
        if (DriverStation.isAutonomous()) {
            driveLimit = CurrentLimits.Drive.autoLimitAmps;
        }
        if (driveState == MechanismStates.Drive.DISABLED) {
            driveLimit = CurrentLimits.Drive.minLimitAmps;
        }
    }

    private void applyStaticLimits() {
        indexerLimit = CurrentLimits.Static.indexerSupplyAmps;
        extenderLimit = CurrentLimits.Static.extenderSupplyAmps;
        rollerLimit = CurrentLimits.Static.rollerSupplyAmps;
        driveLimit = CurrentLimits.Static.driveSupplyAmps;
    }

    private interface BudgetSetter {
        void set(double value);
    }

    private static double fillToward(double remaining, double current, double target, BudgetSetter setter) {
        double add = Math.min(Math.max(0.0, target - current), remaining);
        setter.set(current + add);
        return remaining - add;
    }

    private static double quantize(double amps) {
        return Math.floor(amps / CurrentLimits.Shared.stepAmps) * CurrentLimits.Shared.stepAmps;
    }

    public double getDriveLimit() {
        return driveLimit;
    }

    public double getIndexerLimit() {
        return indexerLimit;
    }

    public double getExtenderLimit() {
        return extenderLimit;
    }

    public double getRollerLimit() {
        return rollerLimit;
    }

    public double getBatteryVoltage() {
        return inputs.batteryVoltage;
    }

    @AutoLog
    public static class BatteryIOInputs {
        public double batteryVoltage = 12.0;
        public double rioCurrent = 0.0;
        public boolean brownedOut = false;
    }
}
