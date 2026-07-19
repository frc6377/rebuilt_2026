# Dynamic Current Limiting for Drive, Intake, and Indexer

Status: Implemented on `brownout-playground-self`; runtime defaults to `SHADOW`

Scope: Drive propulsion, intake rollers, intake extender, and indexer

Explicit exclusion from dynamic writes: Shooter flywheels, shooter upgoers, and all other robot loads

## Executive Summary

Mechanical Advantage's 2026 power manager is a strong architectural fit for this robot, but our allocation policy should be expanded from one flexible load to four controlled groups:

- four drive propulsion motors;
- two intake roller motors;
- one intake extender motor; and
- one physical indexer motor.

The shooter and its upgoers should keep their existing fixed current limits. Their measured current still reduces the capacity available to the controlled groups, but the power manager must never send a current-limit configuration to a shooter or upgoer motor.

The recommended policy is a state-aware priority allocator. It protects an active indexer and moving extender, gives the intake rollers a lower priority, and uses drive propulsion as the main flexible load. This preserves the shot mechanism while allowing the robot to shed current from systems that can tolerate temporary performance loss.

This branch adopts 6328's budgeting and asynchronous-configuration architecture. It carries 6328's MK Powered battery
fit only as a clearly labeled reference seed, not as a characterization of our battery fleet. Telemetry and allocation
run in `SHADOW` by default, so deploying this branch does not stream any dynamic Talon configuration until a human
explicitly selects `ACTIVE`.

## Implementation Status

The code milestone described by this report is complete:

- one `PowerBudgetManager` controls four typed groups and has no shooter or upgoer dependency;
- exactly eight real Talons own current-limit workers: four drive propulsion, two intake rollers, one extender, and one
  indexer;
- the extender also owns a separate neutral-mode worker because its existing brake/coast changes must not perform
  Phoenix configuration calls on the robot thread; both extender workers serialize configuration through one
  per-device lock;
- the unused second indexer owner was removed from `Superstructure`;
- whole-robot current comes from WPILib's automatic PDP/PDH selection;
- drive, extender, and indexer supply-current signals run at 50 Hz; the existing roller signals remain at 50 Hz;
- `OFF`, `SHADOW`, and `ACTIVE` modes are available through `PowerManagement/Mode`;
- `ACTIVE` writes are permitted only while teleop is enabled;
- current-signal refresh status, power-distribution plausibility, and a controlled-versus-total-current mismatch guard
  prevent invalid samples from driving the allocator;
- leaving `ACTIVE` or disabling queues the known startup limits once, while a persistent active-mode input/model fault
  queues them and latches the manager `OFF` until the requested mode exits `ACTIVE`; and
- 70 automated Gradle tests cover model math, allocation priority, recovery behavior, activity state, manager fallback,
  and asynchronous worker concurrency.

Mode behavior:

| Mode | Calculates and logs? | Sends dynamic requests? |
| --- | --- | --- |
| `OFF` | Yes, for diagnostics | No; queues startup-limit restoration after `ACTIVE` |
| `SHADOW` | Yes | No; startup default |
| `ACTIVE` | Yes | Yes, only to the eight scoped Talons and only during enabled teleop |

`ACTIVE` is a playground/characterization mode, not a match-ready default. Extender and indexer floors, battery
parameters, CAN utilization, and mechanism performance still require real-robot validation.

## Exact Scope

| Load | Physical motors | Dynamic supply limit? | Treatment |
| --- | ---: | --- | --- |
| Drive propulsion | 4 | Yes | Receives the remaining controlled budget and absorbs most reductions. |
| Drive steering | 4 | No | Retains fixed configuration so steering authority is not sacrificed. |
| Intake rollers | 2 | Yes | One group split equally between leader and follower. |
| Intake extender | 1 | Yes | Receives a protected minimum while moving or actively holding. |
| Indexer | 1 | Yes | Receives the highest controlled-load priority while feeding. |
| Shooter flywheels | 4 with current configuration | No | Existing fixed limits remain unchanged. Current is observed only. |
| Shooter upgoers | 2 with current configuration | No | Existing fixed limits remain unchanged. Current is observed only. |
| roboRIO, radio, vision, and other loads | Varies | No | Included in measured uncontrolled current. |

"Drive" in this design means the four propulsion Talons, matching 6328's approach. It does not include the four steering Talons.

## What Is Retained from 6328

6328 calculates a safe whole-robot current budget from both a battery model and a main-breaker model:

```text
safeRobotCurrent =
    min(absoluteCeiling,
        headroom * min(batteryCurrentAtVoltageFloor,
                       breakerCurrentForTimeHorizon))
```

Their implementation then subtracts measured non-drive load and gives the residual to the drivetrain. The reference calculation is in [FinanceDepartment.java lines 105-155](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/bf32451ba4337d02d02078c06d689d863a2d9d83/src/main/java/org/littletonrobotics/frc2026/energy/FinanceDepartment.java#L105-L155).

They also:

- decrease the controlled budget immediately during a brownout;
- increase it slowly during brownout recovery;
- divide a group budget across its motors;
- round requested limits down to reduce configuration traffic; and
- apply `CurrentLimitsConfigs` asynchronously so Phoenix configuration calls cannot stall the robot loop.

The latest-value-wins configuration worker is in [TalonFXCurrentConfigurator.cpp lines 29-104](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/bf32451ba4337d02d02078c06d689d863a2d9d83/src/main/cpp/util/TalonFXCurrentConfigurator.cpp#L29-L104).

## Adapted Budget Calculation

The controlled pool should be derived from measured whole-robot current:

```text
controlledMeasuredCurrent =
    driveSupplyCurrent
    + intakeRollerSupplyCurrent
    + extenderSupplyCurrent
    + indexerSupplyCurrent

uncontrolledMeasuredCurrent =
    max(0, powerDistributionTotalCurrent - controlledMeasuredCurrent)

rawControlledBudget =
    max(0, safeRobotCurrent - uncontrolledMeasuredCurrent)
```

`uncontrolledMeasuredCurrent` includes the shooter, upgoers, steering motors, control system, radio, vision hardware, and every other load outside this design. This is how the shooter influences the budget without being throttled.

The subtraction must be logged because PDP/PDH and Talon measurements are not sampled simultaneously. Negative results should be clamped to zero, and implausible differences should raise a diagnostic rather than silently corrupting the allocation.

After any brownout, the controlled pool falls immediately when required but recovers with a tuned upward slew rate.
Recovery retains a monotonic high-water target: a temporary downward raw-budget sample cannot end recovery and allow
the next rebound to jump. The limiter clears recovery only after the minimum recovery window and after the filtered
pool regains that high-water target. Applying the rule to the shared pool, before allocation, is simpler and more
predictable than maintaining four independent brownout controllers.

## State-Aware Priority Allocation

Each controlled group needs:

- an activity state;
- a characterized safe minimum;
- a normal target;
- an absolute maximum;
- a priority; and
- a physical motor count.

Recommended default priority:

| Priority | Group and condition | Reason |
| ---: | --- | --- |
| 1 | Indexer while feeding | A partially powered feed can interrupt a shot or leave game pieces in an undesirable position. |
| 2 | Extender while moving or producing meaningful holding output | It should complete commanded motion rather than dwell under an inadequate limit. |
| 3 | Intake rollers while intaking or outtaking | Temporary roller slowdown is preferable to losing the shooter or an in-progress extender move. |
| 4 | Drive propulsion | It is the largest flexible load and therefore absorbs most budget reductions. |

The implemented allocator proceeds as follows:

1. Account for one 0.5 A hardware step on every controlled Talon. If the pool cannot cover this irreducible 4 A
   aggregate floor, report the deficit instead of claiming the requested limits fit inside the pool.
2. Raise a commanded indexer toward its active-safe minimum.
3. Raise a moving or holding extender toward its active-safe minimum.
4. Raise `IDLE` or `ACTIVE` rollers toward their active-safe minimum.
5. Raise drive propulsion toward its standing drivability minimum.
6. Add standby headroom for every mechanism only after the active minima above.
7. Raise active indexer, extender, and rollers toward their normal targets in priority order.
8. Give all remaining current to drive propulsion, up to its configured maximum.
9. Divide group budgets by physical motor count and round down to the configured current step before requesting a
   Talon update.

If the controlled pool is smaller than the sum of all active-safe minimums, the controller cannot satisfy every
requirement. The implemented emergency order is active indexer, active extender, `IDLE`/`ACTIVE` rollers, and drive
propulsion before inactive standby headroom. Active-minimum, standby, and irreducible-floor deficits are logged and
surfaced with an alert because they indicate that the electrical system has already left its normal operating
envelope.

The final minimums, targets, maxima, recovery rate, and quantization step are tuning results, not values to copy from 6328.

## Determining Activity

Activity must come from commanded state, not only measured current. Waiting for measured current makes the controller react after the load has already started.

- **Drive:** Track the most recent commanded module or chassis speed. Keep a standing drivability reserve while enabled.
- **Intake rollers:** Use the `BaseShooter` roller setpoint and distinguish stopped, idle, and active intake/outtake operation.
- **Extender:** Treat it as active when PID or manual output is enabled and it is either away from target or producing meaningful holding output.
- **Indexer:** Track every output path through one commanded-state field. The current `Indexer.setpoint` is not updated by all percent-output commands, so the existing `Indexer/Running` log is not a sufficient allocator input.

Inactive mechanisms should retain a tuned low standby limit rather than immediately returning to maximum. This prevents a newly commanded mechanism from creating an unrestricted first-cycle surge while the asynchronous configurator catches up. The standby value must still be high enough for reliable startup.

## Current Codebase Mapping

### Drive

The drive Talons currently have a 50 A supply limit enabled and a 60 A effective stator/slip limit in [`TunerConstants.java`](../src/main/java/frc/robot/generated/TunerConstants.java) and [`ModuleIOTalonFX.java`](../src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java).

`driveCurrentAmps` remains the 10 Hz stator-current signal. The implementation adds a separate 50 Hz
`driveSupplyCurrentAmps` signal from `getSupplyCurrent()` and preserves the stator limit and original stator-current
log.

Implemented API:

- `ModuleIOInputs.driveSupplyCurrentAmps`;
- `ModuleIO.setDriveSupplyCurrentLimit(double amps)`;
- a module supply-current getter;
- a module limit setter; and
- drive methods that sum all four propulsion supply currents and apply one per-motor request.

### Intake Rollers

The active real intake implementation is not `PIDRollerIOReal`; [`RobotContainer.java`](../src/main/java/frc/robot/RobotContainer.java) constructs a `BaseShooterIOKrakenX60` using the intake roller configuration.

This is favorable for telemetry: [`BaseShooterIOKrakenX60.java`](../src/main/java/frc/robot/subsystems/shooter/BaseShooterIOKrakenX60.java) already reads leader and follower supply current at 50 Hz. The intake roller configuration currently enables a 50 A supply limit per motor and a 60 A stator limit.

A supply-limit setter now flows through `BaseShooterIO` and `BaseShooter`, but only the intake construction path opts
into workers. The one-argument constructor used by the left and right shooter remains worker-free. Each physical intake
roller owns its own asynchronous configurator.

### Intake Extender

The extender retains its existing stator-current log and 30 A stator limit. Its startup configuration now explicitly
enables the existing 25 A supply maximum.

Implemented:

- an explicit enabled supply-limit configuration;
- a 50 Hz supply-current signal and log field;
- a dynamic supply-limit setter; and
- one asynchronous configurator for the extender Talon.

The remaining real-robot task is to characterize the minimum current needed to start motion, complete a move, and hold
at the relevant intake angles.

### Indexer

The indexer retains its 25 A stator limit and now starts with an enabled 25 A supply maximum.

Implemented:

- a characterized static supply maximum and enable it at startup;
- a 50 Hz supply-current input;
- a dynamic supply-limit setter;
- a complete commanded-state field updated by every velocity and percent-output path; and
- one asynchronous configurator for the physical indexer Talon.

The duplicate `Superstructure` indexer field and IO construction were removed. `RobotContainer` is now the single
physical owner used by bindings and named commands, so only one configurator can target the indexer Talon.

### Shooter and Upgoers

Do not add dynamic-limit setters to the power manager for:

- left or right shooter leader;
- left or right shooter follower; or
- either upgoer.

The shooter currently uses fixed 50 A supply limits per flywheel motor, and the upgoers use fixed 35 A supply limits. Those configurations remain unchanged.

Their supply-current signals are useful for logging, but the allocator can remain decoupled from shooter classes by obtaining uncontrolled current from:

```text
PDP/PDH total current - measured controlled-load current
```

An optional later improvement is a feed-forward shooter reserve when a shooter setpoint becomes active. That would preemptively reduce the controlled pool during spin-up without changing any shooter current limit.

## Implemented Components

### `energy/PowerIO`

Log:

- battery voltage;
- PDP/PDH total current;
- roboRIO brownout state;
- device connection/status; and
- sample timestamp or age where available.

`PowerIOReal` uses `new PowerDistribution()`, which
[asks WPILib to detect](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/PowerDistribution.html)
a PDP or PDH at the vendor's default CAN ID. This avoids guessing hardware from source. Historical logs report 24
channels, but the detected module/type, current validity, and agreement with robot measurements must still be checked
on the physical robot before `ACTIVE` use.

### `energy/BatteryEstimator` and `energy/BreakerModel`

The model structure and MK Powered reference fit are adapted from 6328. Their MIT notice is retained in the derived
source, and the full license is recorded in [`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md).

The reference coefficients are isolated behind `mechanicalAdvantageMkPoweredReference()` and logged as
uncharacterized. They are not final values; our fleet requires log fitting and load testing.

### `energy/PowerBudgetManager`

Implemented responsibilities:

- update the battery and breaker models;
- calculate the safe robot budget;
- derive the controlled pool;
- apply brownout recovery behavior;
- evaluate activity states;
- reject invalid/stale controlled-current samples and implausible controlled-versus-PDP/PDH current mismatch;
- allocate group budgets in priority order;
- quantize per-motor limits;
- publish requested limits; and
- log every intermediate value and limiting reason.

Logged outputs:

- safe robot budget;
- battery-limited and breaker-limited budgets;
- measured total, controlled, and uncontrolled current;
- raw and recovery-filtered controlled pool;
- activity and priority state for every group;
- requested group and per-motor limits;
- active-minimum, standby, and irreducible-floor deficits;
- active-input fault duration and latch state; and
- current limiting reason.

In `ACTIVE`, a single invalid sample holds the last dynamic requests rather than generating a configuration burst. If
invalid current data, excessive measurement mismatch, or a model exception persists for 0.1 seconds, the manager queues
the startup limits once and latches its effective mode to `OFF`. The latch clears only after the requested mode leaves
`ACTIVE` or the manager resets. The snapshot/log field `StartupLimitsRestored` records that the restore request was
queued; per-worker requested-versus-last-successfully-acknowledged telemetry records whether Phoenix accepted it.

### `util/TalonFXCurrentConfigurator`

The implementation uses one helper per physical controlled Talon:

- four drive;
- two intake rollers;
- one extender; and
- one indexer.

Each helper:

- queues the complete startup current configuration as its first revision and retries it until acknowledged;
- skips identical current-limit configurations;
- keep only the latest pending request;
- copy mutable Phoenix configuration data before crossing threads;
- release its lock before calling Phoenix;
- use the normal/default `apply(CurrentLimitsConfigs)` timeout;
- retry failed writes at a 100 ms interval;
- allow a new request to supersede a retry;
- preserve stator and lower-limit fields while changing only the supply limit; and
- log requested limit, last successful limit, status, retry count, and apply age.

Phoenix's [current-limit API](https://api.ctr-electronics.com/phoenix6/latest/java/com/ctre/phoenix6/configs/CurrentLimitsConfigs.html)
defines the lower-limit stage independently, so streaming requests intentionally preserve it along with the stator
configuration.

Eight configurable devices create more CAN traffic than 6328's four drive motors. The implementation combines 0.5 A
quantization and deduplication with a 100 ms minimum interval for nonurgent writes. Production workers receive
deterministic 3 ms offsets across eight slots so increases do not all become eligible at once. Urgent decreases bypass
both the interval and the stagger. A newer value superseding an already-pending increase retains the original
eligibility time, preventing a 20 ms request stream from continually restarting a longer stagger. Real-robot CAN
utilization is still an acceptance gate.

The extender's pre-existing brake/coast changes use a ninth background thread, but not a ninth controlled device. That
neutral-mode worker clones the complete startup `MotorOutputConfigs`, retries failed writes, and shares a per-device
configuration lock with the extender current worker.

## Scheduling

`Robot.robotPeriodic()` now calls `robotContainer.updatePowerManagement()` immediately after
`CommandScheduler.run()`.

That method:

1. updates and logs `PowerIO`;
2. reads controlled-load supply currents and command states collected during subsystem periodic methods;
3. validates all required samples;
4. updates the battery and breaker models;
5. calculates the four group budgets;
6. publishes shadow or active requests; and
7. sends changed requests to the asynchronous configurators.

This order lets the manager see commands issued during the current scheduler cycle and keeps every Phoenix configuration call off the main robot thread.

## Shooter-Isolation Invariants

The implementation, production wiring, source checks, and tests enforce these invariants:

1. The controlled-load registry contains only drive propulsion, intake rollers, extender, and the single physical indexer.
2. The power manager has no shooter or upgoer constructor dependency or output path.
3. Only the intake construction of the shared roller/flywheel IO opts into current-limit workers; the one-argument
   constructor used by both shooter sides remains worker-free.
4. Shooter and upgoer current affects the allocator only as part of measured uncontrolled current or an explicitly read-only reserve.
5. Disabling dynamic power management queues a known static startup limit for every controlled motor.
6. Persistent invalid manager input queues the known startup limits; each failed worker retains its last acknowledged
   limit and retries without blocking the robot loop.

## Rollout Plan

### Phase 0: Ownership and Power Source

- [x] Remove the duplicate indexer hardware owner.
- [x] Use WPILib automatic PDP/PDH selection instead of guessing a module/type.
- [x] Define 25 A startup supply maxima for extender and indexer.
- [ ] Confirm the automatically detected hardware and current data on the physical robot.

### Phase 1: Telemetry

- [x] Add 50 Hz supply-current signals for drive, extender, and indexer.
- [x] Reuse the intake roller's existing 50 Hz supply signals.
- [x] Log controlled, uncontrolled, mismatch, model, allocation, and worker state.
- [x] Keep runtime in `SHADOW`, with no dynamic writes.

### Phase 2: Configurator Validation

- [x] Unit-test startup-baseline acknowledgement, deduplication, latest-value-wins behavior, sustained failure, retry,
  exception survival, nonstarving pacing/stagger, and field preservation.
- [ ] Apply manually selected limits to one controlled device at a time on the robot.
- [ ] Verify requested versus last-successfully-acknowledged state, main-loop timing, and CAN utilization.
- [ ] Confirm through hardware logs that shooter and upgoer configurations never change.

### Phase 3: Shadow Allocation

- [x] Run the battery, breaker, recovery, and priority calculations without writing their results by default.
- [ ] Replay or analyze matches containing simultaneous driving, shooting, intake, extension, and indexing.
- [ ] Tune activity detection, floors, targets, and battery parameters.

### Phase 4: Guarded Teleop

- [x] Provide an explicit `ACTIVE` runtime mode while defaulting to `SHADOW`.
- [x] Permit `ACTIVE` writes only during enabled teleop.
- [x] Queue startup-limit restoration whenever the robot is disabled or leaves `ACTIVE`.
- [x] Hold the last dynamic request through a 0.1-second input-fault grace, then queue startup limits and latch `OFF`
  for persistent input/model faults.
- [ ] Enable for guarded teleop testing only after Phases 2 and 3 pass on hardware.
- [ ] Begin with narrow dynamic ranges around verified static limits.

### Phase 5: Autonomous Evaluation

- [ ] Enable in autonomous only after path tracking, intake timing, indexer behavior, and shot readiness remain
  repeatable.

## Verification Scenarios

The branch's 70 automated tests cover:

- MK Powered battery-model regression points, breaker trip/damage behavior, and input validation;
- residual controlled-pool calculation and asynchronous measurement mismatch;
- irreducible-floor, standby, full-budget, severe-deficit, emergency-priority, and quantization behavior;
- immediate reductions, rate-limited brownout recovery, and dip-then-rebound high-water recovery;
- `SHADOW` versus teleop-only `ACTIVE`, one-time startup restoration, transient/persistent invalid input, model
  exceptions, fault re-arming, current mismatch, and shooter/upgoer constructor isolation;
- indexer command activity, stopped/idle/active roller classification, and first-cycle/measured-output extender
  activity;
- power-distribution sample plausibility; and
- asynchronous startup acknowledgement, field preservation, defensive copies, deduplication, latest-value-wins
  ordering, sustained failure, retry, exception survival, urgent-decrease pacing, nonstarving device stagger,
  telemetry age, and shutdown.

The following remain hardware verification scenarios:

1. Shooter off and no mechanisms active: drive can receive its full configured maximum.
2. Shooter spin-up: shooter limits remain unchanged while the controlled pool decreases.
3. Shooter plus indexer: the indexer retains its active-safe allocation before drive receives residual current.
4. Intake and indexing together: extender and indexer floors are protected, rollers receive their active allocation, and drive absorbs the remaining reduction.
5. All controlled loads active with insufficient budget: the emergency priority order and alert are deterministic.
6. Brownout recovery: every controlled limit can fall immediately and the total pool rises only at the configured recovery rate.
7. Stale or mismatched current samples: transient faults hold the last dynamic request, persistent faults queue the
   startup maxima and latch `OFF`, and negative uncontrolled current is clamped and diagnosed.
8. Requested and last-successfully-acknowledged values converge on all eight Talons.
9. Main-loop timing and CAN error counts remain acceptable while requests change.
10. Configuration failure is visible in logs while the last known limit remains active.
11. Shooter and upgoer configurations remain fixed in every scenario.
12. Before simultaneous high-load testing, operators wait after the first transition into `ACTIVE` until all eight
    requested/current acknowledgements converge.

Robot acceptance criteria should include:

- no new loop overruns;
- no sustained increase in CAN errors;
- requested and last-successfully-acknowledged limits converge;
- fewer `/SystemStats/BrownedOut` rising edges;
- no measurable degradation in shooter ready time or velocity regulation;
- reliable indexer feeding without new stalls or jams;
- acceptable extender motion time and holding behavior; and
- acceptable drive tracking and driver feel.

## Risks and Limitations

- A current limit is a ceiling, not a reservation. Unused allocated current is recovered only when the manager observes it and recalculates.
- The first `ACTIVE` request is asynchronous. Until each worker acknowledges it, a motor may still have its startup
  maximum, so guarded tests must wait for requested/acknowledged convergence before applying simultaneous loads.
- The controller cannot prevent the first instantaneous surge from the fixed-limit shooter or from a mechanism whose
  new request has not yet reached its Talon.
- During established `ACTIVE` operation, an inactive indexer, extender, or roller can sit at the uncharacterized 3 A
  standby request. Its increase to the 10 A active minimum is nonurgent and can wait for the device stagger, the
  remaining portion of the 100 ms per-device interval, and the Phoenix call. Hardware testing must show this does not
  stall or jam a newly activated mechanism.
- PDP/PDH and Talon current samples are asynchronous; subtraction will contain measurement error.
- Persistent invalid input intentionally fails open by queuing the known static startup maxima and latching `OFF`. This
  requests pre-feature behavior but removes dynamic brownout protection until the operator exits `ACTIVE` and re-arms
  it; failed configuration writes can leave the previously acknowledged cap in effect meanwhile.
- If the safe budget falls below all active-safe floors, current limiting alone cannot make every mechanism perform correctly.
- Poorly tuned extender or indexer minima can cause stalls, long dwell times, or inconsistent game-piece motion.
- Frequent configuration changes can consume CAN bandwidth. At the fixed 100 ms pace, sustained changes or failures
  can attempt roughly 80 current configurations per second across eight workers, plus extender neutral-mode traffic;
  the apply rate and CAN errors must be measured on the real robot.
- The extender current and neutral-mode workers share one device lock. An urgent current decrease can wait behind one
  in-progress neutral-mode configuration call, whose duration is bounded by Phoenix's default timeout but still needs
  real-hardware measurement.
- A successful Phoenix `apply` is logged as an acknowledgement, not as an independent device readback.
- The implementation adds roughly 216 logging calls per 20 ms cycle across the manager and nine helper snapshots;
  loop time and log volume must be measured on the roboRIO.
- Battery-model accuracy depends on battery chemistry, condition, temperature, and fitted parameters.
- 6328 runs a 5 ms control loop with 200 Hz module supply-current signals. Our 20 ms loop should be evaluated with at least 50 Hz supply-current telemetry rather than assuming identical behavior.

## Recommendation

Keep this branch in `SHADOW` while collecting simultaneous drive, shooting, intake, extender, and indexer logs. The code
milestone is complete, but match use remains gated on physical PDP/PDH verification, battery fitting, mechanism-floor
characterization, CAN utilization, and repeatable guarded-teleop tests.

When those gates pass, enable `ACTIVE` first for controlled teleop trials. The shooter and upgoers remain fixed
throughout: they consume the measured residual budget but never receive a dynamic current-limit command.
