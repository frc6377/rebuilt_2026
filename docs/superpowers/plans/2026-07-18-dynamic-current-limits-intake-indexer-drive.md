# Drive, Intake, and Indexer Dynamic Current Limits Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use test-driven development task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement Mechanical Advantage-inspired current budgeting for drive propulsion, both intake rollers, the intake extender, and the single physical indexer while structurally preventing dynamic writes to shooter and upgoer motors.

**Architecture:** Pure Java battery, breaker, recovery, and typed load-allocation classes produce per-motor limits for exactly four controlled groups. `PowerBudgetManager` reads automatic PDP/PDH total current plus controlled Talon supply currents after the command scheduler, logs shadow results, and conditionally forwards them through latest-value-wins background Phoenix configuration workers. Runtime defaults to shadow mode because extender/indexer active floors are not characterized.

**Tech Stack:** Java 17, WPILib 2026.2.1, AdvantageKit, Phoenix 6 vendor version 26.1.0, JUnit Jupiter 5.10.1, GradleRIO.

---

## Scope and invariants

- Controlled: four drive propulsion Talons, two intake roller Talons, one extender Talon, one physical indexer Talon.
- Not controlled: four swerve steering Talons, four shooter flywheel Talons, two upgoer Talons, and all other loads.
- Shooter/upgoer current affects the residual pool only through whole-robot current.
- Dynamic workers clone fully initialized current configurations and mutate only `SupplyCurrentLimit`.
- Fixed stator limits remain unchanged.
- `SHADOW` is the startup mode; no dynamic writes occur until explicitly enabled.
- A single `IndexerIOReal` owns the physical indexer.

## Task 1: Pure energy models and budget math

**Files:**

- Create: `src/main/java/frc/robot/energy/BatteryEstimator.java`
- Create: `src/main/java/frc/robot/energy/BreakerModel.java`
- Create: `src/main/java/frc/robot/energy/BudgetMath.java`
- Test: `src/test/java/frc/robot/energy/BatteryEstimatorTest.java`
- Test: `src/test/java/frc/robot/energy/BreakerModelTest.java`
- Test: `src/test/java/frc/robot/energy/BudgetMathTest.java`

- [ ] Write regression tests for the Mechanical Advantage MK Powered reference parameters, voltage-floor maximum current, breaker trip points, damage/cooling, and residual controlled-pool derivation.
- [ ] Run:

  ```powershell
  $env:JAVA_HOME='C:\Users\Public\wpilib\2026\jdk'
  .\gradlew.bat test --tests "frc.robot.energy.*"
  ```

  Expected: compile failure because the three production classes do not exist.

- [ ] Implement models with explicit `dtSeconds`, finite-value validation, no logger dependency, and immutable battery parameters exposed through `mechanicalAdvantageMkPoweredReference()`.
- [ ] Preserve the source MIT notice on substantially derived model formulas and breaker curve points.
- [ ] Re-run the targeted tests; expect all energy-model tests to pass.

Required public behavior:

```java
BatteryEstimator estimator =
        new BatteryEstimator(BatteryEstimator.Parameters.mechanicalAdvantageMkPoweredReference());
estimator.reset(terminalVoltage, currentAmps);
estimator.update(totalCurrentAmps, terminalVoltage, dtSeconds);
double batteryLimit = estimator.maxCurrentAtVoltage(minimumVoltage);

BreakerModel breaker = new BreakerModel(0.05);
breaker.update(totalCurrentAmps, dtSeconds);
double breakerLimit = breaker.maxCurrentFor(horizonSeconds);

BudgetMath.Result result =
        BudgetMath.derive(safeRobotCurrent, pdTotalCurrent, controlledMeasuredCurrent);
```

## Task 2: Typed state-aware allocator

**Files:**

- Create: `src/main/java/frc/robot/energy/StateAwareCurrentAllocator.java`
- Test: `src/test/java/frc/robot/energy/StateAwareCurrentAllocatorTest.java`

- [ ] Write failing tests for standby allocation, full allocation, severe deficit priority, quantization without oversubscription, invalid policy ordering, invalid motor count, and non-finite/negative pools.
- [ ] Verify RED with:

  ```powershell
  .\gradlew.bat test --tests "frc.robot.energy.StateAwareCurrentAllocatorTest"
  ```

- [ ] Implement named records rather than a generic subsystem map:

  ```java
  record LoadPolicy(
          int motorCount,
          double standbyPerMotor,
          double activeMinimumPerMotor,
          double targetPerMotor,
          double maximumPerMotor) {}

  record AllocationConfig(
          LoadPolicy drive,
          LoadPolicy rollers,
          LoadPolicy extender,
          LoadPolicy indexer,
          double currentStepAmps) {}

  record Activity(boolean rollersActive, boolean extenderActive, boolean indexerActive) {}

  record Allocation(
          double drivePerMotor,
          double rollersPerMotor,
          double extenderPerMotor,
          double indexerPerMotor,
          double allocatedAmps,
          double unusedAmps,
          double activeMinimumDeficitAmps,
          boolean standbyDeficit) {}
  ```

- [ ] Allocate whole group quanta in this order: standby, active indexer minimum, active extender minimum, active roller minimum, drive minimum, active mechanism targets, then drive maximum.
- [ ] Re-run the allocator tests and all energy tests; expect PASS.

## Task 3: Asynchronous Phoenix current configurator

**Files:**

- Create: `src/main/java/frc/robot/util/TalonFXCurrentConfigurator.java`
- Test: `src/test/java/frc/robot/util/TalonFXCurrentConfiguratorTest.java`

- [ ] Write failing tests using an injected `ConfigApplier`, `CountDownLatch`, and Phoenix `StatusCode` values for:
  - preservation of all six current-limit fields;
  - defensive cloning;
  - equal-request deduplication;
  - A-in-flight/B/C latest-wins ordering;
  - retry after failure;
  - newer request superseding retry;
  - exception survival;
  - urgent decrease bypassing minimum interval;
  - idempotent close; and
  - invalid/request-after-close rejection.
- [ ] Verify RED with the targeted util test.
- [ ] Implement `AutoCloseable` worker with one lock/condition, daemon low-priority thread, default-timeout `TalonFXConfigurator.apply(CurrentLimitsConfigs)`, 20 ms retry, latest revision tracking, and immutable snapshot telemetry.
- [ ] Construct every request from a clone of the fully initialized baseline and change only `SupplyCurrentLimit`.
- [ ] Re-run the targeted helper tests; expect PASS.

## Task 4: Controlled-load IO telemetry and setters

**Files:**

- Modify: `src/main/java/frc/robot/subsystems/drive/ModuleIO.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/Module.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/Drive.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFXReal.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/BaseShooterIO.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/BaseShooter.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/BaseShooterIOKrakenX60.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/Intake.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/extender/ExtenderIO.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/extender/Extender.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/extender/ExtenderIOReal.java`
- Modify: `src/main/java/frc/robot/subsystems/indexer/IndexerIO.java`
- Modify: `src/main/java/frc/robot/subsystems/indexer/Indexer.java`
- Modify: `src/main/java/frc/robot/subsystems/indexer/IndexerIOReal.java`
- Modify: `src/main/java/frc/robot/subsystems/indexer/IndexerIOSim.java`
- Modify: `src/main/java/frc/robot/subsystems/indexer/IndexerConstants.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`
- Test: `src/test/java/frc/robot/subsystems/indexer/IndexerActivityTest.java`

- [ ] Add testable subsystem activity-state tests before changing `Indexer`; verify current percent-output paths fail to update a complete state.
- [ ] Remove the unused second indexer, its IO switch, and imports from `Superstructure`.
- [ ] Add separate 50 Hz supply-current fields without renaming existing stator-current log fields.
- [ ] Add current-limit forwarding methods to the four controlled groups.
- [ ] Opt only the intake-owned `BaseShooterIOKrakenX60` into two configurator workers; the existing one-argument constructor used by shooter remains fixed and worker-free.
- [ ] Enable a static 25 A indexer supply maximum and the existing 25 A extender supply maximum while preserving their 25 A and 30 A stator limits.
- [ ] Track every indexer velocity/percent/stop path through one activity state.
- [ ] Expose extender activity from PID/manual state rather than command lifetime.
- [ ] Run `compileJava` and the full test suite; expect PASS.

Required facade:

```java
drive.getDriveSupplyCurrentAmps();
drive.setDriveSupplyCurrentLimit(perMotorAmps);

intake.getRollerSupplyCurrentAmps();
intake.isRollerRunning();
intake.setRollerSupplyCurrentLimit(perMotorAmps);
intake.getExtenderSupplyCurrentAmps();
intake.isExtenderActiveForPowerManagement();
intake.setExtenderSupplyCurrentLimit(amps);

indexer.getSupplyCurrentAmps();
indexer.isPowerManagementActive();
indexer.setSupplyCurrentLimit(amps);
```

## Task 5: Power IO, recovery, and manager orchestration

**Files:**

- Create: `src/main/java/frc/robot/energy/PowerIO.java`
- Create: `src/main/java/frc/robot/energy/PowerIOReal.java`
- Create: `src/main/java/frc/robot/energy/PowerIOSim.java`
- Create: `src/main/java/frc/robot/energy/BrownoutRecoveryLimiter.java`
- Create: `src/main/java/frc/robot/energy/PowerBudgetManager.java`
- Create: `src/main/java/frc/robot/energy/EnergyConstants.java`
- Test: `src/test/java/frc/robot/energy/BrownoutRecoveryLimiterTest.java`
- Test: `src/test/java/frc/robot/energy/PowerBudgetManagerTest.java`

- [ ] Write failing tests proving immediate budget decreases, delayed recovery increases, controlled-current subtraction, typed priority outputs, fallback restoration, and zero shooter/upgoer dependencies.
- [ ] Implement `PowerIOReal` with `new PowerDistribution()` automatic module/type selection plus `RobotController` voltage and brownout state.
- [ ] Define `OFF`, `SHADOW`, and `ACTIVE` modes; default to `SHADOW`.
- [ ] Seed shadow model constants from the clearly labeled 6328 reference values, not as characterized robot values.
- [ ] Keep all manager outputs named by controlled group rather than a generic motor registry.
- [ ] In `ACTIVE`, fan out only the four typed results. In `OFF`/`SHADOW`, do not issue dynamic writes; when leaving `ACTIVE`, restore static startup limits once.
- [ ] Log model state, measured totals, residual pool, activities, allocations, deficits, worker status, and limiting reason from the robot thread.
- [ ] Re-run targeted and full tests.

## Task 6: Robot scheduling and generated logging

**Files:**

- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Modify: `src/main/java/frc/robot/Robot.java`
- Generated by annotation processing: `*InputsAutoLogged` classes

- [ ] Instantiate the manager after the real/sim IO switch has assigned drive, intake, and indexer.
- [ ] Add `RobotContainer.updatePowerManagement()` and call it immediately after `CommandScheduler.run()` so the manager sees the current cycle's commands.
- [ ] Add reset handling on disable/startup without changing shooter or upgoer construction.
- [ ] Run Spotless, tests, and build:

  ```powershell
  .\gradlew.bat spotlessApply test build
  ```

- [ ] Inspect `git diff` for generated/unrelated changes and retain only feature-scoped files.

## Task 7: Documentation, parity, and final verification

**Files:**

- Update: `docs/dynamic-current-limits-intake-indexer-drive.md`
- Update: `docs/superpowers/plans/2026-07-18-dynamic-current-limits-intake-indexer-drive.md`

- [ ] Mark implementation status and record that runtime defaults to shadow pending real-robot characterization.
- [ ] Verify no dynamic setter or worker is constructed for shooter/upgoer code paths with source search and tests.
- [ ] Run:

  ```powershell
  .\gradlew.bat spotlessCheck test build
  git diff --check
  git status --short --branch
  ```

- [ ] Review requested/applied naming: Phoenix OK is an acknowledged write, not an independent readback.
- [ ] Record residual real-robot gates: characterize extender/indexer floors, verify PDP/PDH current, measure CAN utilization, and compare `/SystemStats/BrownedOut` rising edges.
