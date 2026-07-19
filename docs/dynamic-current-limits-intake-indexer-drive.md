# Dynamic Current Limiting for Drive, Intake, and Indexer

Status: Proposed design

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

Adopt 6328's budgeting and asynchronous-configuration architecture, not its battery parameters or current-limit values. Begin with telemetry and shadow calculations before enabling any dynamic motor configuration.

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

After any brownout, the controlled pool should fall immediately when required but recover with a tuned upward slew rate. Applying the recovery rule to the shared pool, before allocation, is simpler and more predictable than maintaining four independent brownout controllers.

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

Allocation should proceed as follows:

1. Reserve the tuned active-safe minimum for the indexer when it is commanded.
2. Reserve the tuned active-safe minimum for the extender while it is moving or actively holding.
3. Reserve the roller minimum appropriate to `OFF`, `IDLE`, or `ACTIVE`.
4. Reserve a minimum per-drive-motor limit that preserves basic drivability.
5. Distribute remaining current toward each mechanism's normal target in priority order.
6. Give all remaining current to drive propulsion, up to its configured maximum.
7. Divide group budgets by physical motor count and round down to the configured current step before requesting a Talon update.

If the controlled pool is smaller than the sum of all active-safe minimums, the controller cannot satisfy every requirement. The recommended emergency order is indexer while feeding, extender while moving, intake rollers, then drive propulsion. This condition must be logged and surfaced with an alert because it indicates that the electrical system has already left its normal operating envelope.

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

However, `driveCurrentAmps` currently comes from `getStatorCurrent()` and is updated at only 10 Hz. Add a separate `driveSupplyCurrentAmps` signal from `getSupplyCurrent()` at no less than the 50 Hz robot-loop rate. Preserve the existing stator limit and stator-current log.

Required API additions:

- `ModuleIOInputs.driveSupplyCurrentAmps`;
- `ModuleIO.setDriveSupplyCurrentLimit(double amps)`;
- a module supply-current getter;
- a module limit setter; and
- drive methods that sum all four propulsion supply currents and apply one per-motor request.

### Intake Rollers

The active real intake implementation is not `PIDRollerIOReal`; [`RobotContainer.java`](../src/main/java/frc/robot/RobotContainer.java) constructs a `BaseShooterIOKrakenX60` using the intake roller configuration.

This is favorable for telemetry: [`BaseShooterIOKrakenX60.java`](../src/main/java/frc/robot/subsystems/shooter/BaseShooterIOKrakenX60.java) already reads leader and follower supply current at 50 Hz. The intake roller configuration currently enables a 50 A supply limit per motor and a 60 A stator limit.

Add a supply-limit setter through `BaseShooterIO` and `BaseShooter`, but call it only on the `BaseShooter` instance owned by `Intake`. Shooter instances share this IO class and must never be registered as controlled loads. Each intake roller Talon needs its own asynchronous current configurator.

### Intake Extender

The extender currently logs stator current. Its configuration assigns a 25 A supply-limit value but does not set `SupplyCurrentLimitEnable`; the current code therefore does not establish that the 25 A supply limit is active.

Add:

- an explicit enabled supply-limit configuration;
- a 50 Hz supply-current signal and log field;
- a dynamic supply-limit setter; and
- one asynchronous configurator for the extender Talon.

Keep the existing 30 A stator limit unchanged. Characterize the minimum current needed to start motion, complete a move, and hold at the relevant intake angles.

### Indexer

The indexer currently has a 25 A stator limit but no supply-current limit and no current field in `IndexerIOInputs`.

Add:

- a characterized static supply maximum and enable it at startup;
- a 50 Hz supply-current input;
- a dynamic supply-limit setter;
- a complete commanded-state field updated by every velocity and percent-output path; and
- one asynchronous configurator for the physical indexer Talon.

There is an ownership issue that must be resolved before dynamic limiting: both [`RobotContainer.java`](../src/main/java/frc/robot/RobotContainer.java) and [`Superstructure.java`](../src/main/java/frc/robot/subsystems/superstructure/Superstructure.java) currently construct an `IndexerIOReal` using the same configured CAN ID. The `Superstructure` indexer field is not otherwise used in the current source. Retain one physical owner, preferably the `RobotContainer` instance already used by bindings and named commands, and remove or inject that instance into `Superstructure`. Two configuration workers must never target the same Talon.

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

## Proposed Components

### `energy/PowerIO`

Log:

- battery voltage;
- PDP/PDH total current;
- roboRIO brownout state;
- device connection/status; and
- sample timestamp or age where available.

The installed PDP/PDH type and CAN ID are not defined in the current repository and must be confirmed on the robot before this is implemented.

### `energy/BatteryEstimator` and `energy/BreakerModel`

Port or independently implement the model structure from 6328. Their 2026 code is MIT-licensed, so copied or substantially derived source should retain the MIT notice.

Do not copy the estimator coefficients as final values. 6328 states that its battery parameters were fitted for an MK Powered battery. Our values require fleet-specific log fitting and load testing.

### `energy/PowerBudgetManager`

Responsibilities:

- update the battery and breaker models;
- calculate the safe robot budget;
- derive the controlled pool;
- apply brownout recovery behavior;
- evaluate activity states;
- allocate group budgets in priority order;
- quantize per-motor limits;
- publish requested limits; and
- log every intermediate value and limiting reason.

Suggested outputs:

- safe robot budget;
- battery-limited and breaker-limited budgets;
- measured total, controlled, and uncontrolled current;
- raw and recovery-filtered controlled pool;
- activity and priority state for every group;
- requested group and per-motor limits;
- budget-deficit amount; and
- current limiting reason.

### `util/TalonFXCurrentConfigurator`

Use one helper per physical controlled Talon:

- four drive;
- two intake rollers;
- one extender; and
- one indexer.

Each helper should:

- skip identical current-limit configurations;
- keep only the latest pending request;
- copy mutable Phoenix configuration data before crossing threads;
- release its lock before calling Phoenix;
- use the normal/default `apply(CurrentLimitsConfigs)` timeout;
- retry failed writes after a short delay;
- allow a new request to supersede a retry;
- preserve stator and lower-limit fields while changing only the supply limit; and
- log requested limit, last successful limit, status, retry count, and apply age.

Eight configurable devices create more CAN traffic than 6328's four drive motors. Quantization, deduplication, and a tunable minimum apply interval are therefore required. Urgent decreases should apply immediately; increases may be staggered if CAN measurements show that simultaneous updates are costly.

## Scheduling

Call a method such as `robotContainer.updatePowerManagement()` immediately after `CommandScheduler.run()` in [`Robot.java`](../src/main/java/frc/robot/Robot.java).

That method should:

1. update and log `PowerIO`;
2. read controlled-load supply currents and command states collected during subsystem periodic methods;
3. update the battery and breaker models;
4. calculate the four group budgets;
5. publish shadow or active requests; and
6. send changed requests to the asynchronous configurators.

This order lets the manager see commands issued during the current scheduler cycle and keeps every Phoenix configuration call off the main robot thread.

## Shooter-Isolation Invariants

The implementation and tests should enforce these invariants:

1. The controlled-load registry contains only drive propulsion, intake rollers, extender, and the single physical indexer.
2. No shooter or upgoer object receives a dynamic current-limit method call.
3. Shooter and upgoer current affects the allocator only as part of measured uncontrolled current or an explicitly read-only reserve.
4. Disabling dynamic power management restores a known static startup limit for every controlled motor.
5. Failure of the budget manager or a configuration worker leaves a known conservative static limit, never an unbounded motor.

## Rollout Plan

### Phase 0: Ownership and Power Source

- Remove the duplicate indexer hardware owner.
- Confirm PDP/PDH type and CAN ID.
- Define conservative static supply maxima for extender and indexer.

### Phase 1: Telemetry

- Add supply-current signals for drive, extender, and indexer.
- Reuse the intake roller's existing 50 Hz supply signals.
- Log controlled and uncontrolled current.
- Keep all physical limits static.

### Phase 2: Configurator Validation

- Apply manually selected limits to one controlled device at a time.
- Verify requested versus applied state, retry behavior, main-loop timing, and CAN utilization.
- Confirm through logs that shooter and upgoer configurations never change.

### Phase 3: Shadow Allocation

- Run the battery, breaker, recovery, and priority calculations without writing their results.
- Replay or analyze matches containing simultaneous driving, shooting, intake, extension, and indexing.
- Tune activity detection, floors, targets, and battery parameters.

### Phase 4: Guarded Teleop

- Enable behind a feature flag in teleop.
- Retain existing autonomous and disabled behavior.
- Begin with narrow dynamic ranges around verified static limits.

### Phase 5: Autonomous Evaluation

- Enable in autonomous only after path tracking, intake timing, indexer behavior, and shot readiness remain repeatable.

## Verification Scenarios

Unit and integration tests should cover:

1. Shooter off and no mechanisms active: drive can receive its full configured maximum.
2. Shooter spin-up: shooter limits remain unchanged while the controlled pool decreases.
3. Shooter plus indexer: the indexer retains its active-safe allocation before drive receives residual current.
4. Intake and indexing together: extender and indexer floors are protected, rollers receive their active allocation, and drive absorbs the remaining reduction.
5. All controlled loads active with insufficient budget: the emergency priority order and alert are deterministic.
6. Brownout recovery: every controlled limit can fall immediately and the total pool rises only at the configured recovery rate.
7. Stale or mismatched current samples: negative uncontrolled current is clamped and diagnosed.
8. Repeated equal requests: no additional Phoenix configurations are sent.
9. Rapidly changing requests: only the latest request is applied.
10. Configuration failure: retries occur without blocking the robot loop, and the last known safe limit remains active.
11. Shooter isolation: fake shooter/upgoer configurators receive zero calls in every scenario.

Robot acceptance criteria should include:

- no new loop overruns;
- no sustained increase in CAN errors;
- requested and applied limits converge;
- fewer `/SystemStats/BrownedOut` rising edges;
- no measurable degradation in shooter ready time or velocity regulation;
- reliable indexer feeding without new stalls or jams;
- acceptable extender motion time and holding behavior; and
- acceptable drive tracking and driver feel.

The existing [brownout-counter design](superpowers/specs/2026-07-18-robot-brownout-counter-design.md) can provide the event-count metric for before-and-after comparisons.

## Risks and Limitations

- A current limit is a ceiling, not a reservation. Unused allocated current is recovered only when the manager observes it and recalculates.
- The controller cannot fully prevent the first instantaneous surge from a newly started shooter or mechanism.
- PDP/PDH and Talon current samples are asynchronous; subtraction will contain measurement error.
- If the safe budget falls below all active-safe floors, current limiting alone cannot make every mechanism perform correctly.
- Poorly tuned extender or indexer minima can cause stalls, long dwell times, or inconsistent game-piece motion.
- Frequent configuration changes can consume CAN bandwidth; the apply rate must be measured on the real robot.
- Battery-model accuracy depends on battery chemistry, condition, temperature, and fitted parameters.
- 6328 runs a 5 ms control loop with 200 Hz module supply-current signals. Our 20 ms loop should be evaluated with at least 50 Hz supply-current telemetry rather than assuming identical behavior.

## Recommendation

Proceed with the state-aware priority allocator, subject to two prerequisites: consolidate indexer ownership and confirm the installed power-distribution hardware. The first implementation milestone should stop at telemetry and shadow allocation. Only after the asynchronous configurator and priority policy are validated independently should dynamic limits be enabled on drive, both intake roller motors, the extender, and the indexer.

The shooter and upgoers should remain fixed throughout the project. Their role in the power manager is read-only: they consume budget, but they never receive a dynamic current-limit command.
