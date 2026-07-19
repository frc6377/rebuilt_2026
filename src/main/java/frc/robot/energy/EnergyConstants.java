package frc.robot.energy;

import frc.robot.energy.StateAwareCurrentAllocator.AllocationConfig;
import frc.robot.energy.StateAwareCurrentAllocator.LoadPolicy;

/**
 * Initial power-management values.
 *
 * <p>The battery/breaker and recovery values below are explicitly seeded from Mechanical Advantage's 2026 reference
 * implementation. The allocation floors are conservative starting estimates, not characterized Howdy Bots values. Keep
 * runtime in shadow mode until the real-robot rollout gates in the design report are complete.
 */
public final class EnergyConstants {
    public static final String MODEL_SOURCE = "6328 2026 MK Powered reference; not robot-characterized";

    public static final double LOOP_PERIOD_SECONDS = 0.02;
    public static final double MINIMUM_BATTERY_VOLTAGE = 7.0;
    public static final double ABSOLUTE_ROBOT_CURRENT_CEILING_AMPS = 200.0;
    public static final double BUDGET_HEADROOM = 0.9;
    public static final double BREAKER_NICENESS = 0.05;
    public static final double BREAKER_DANGER_HORIZON_SECONDS = 3.0;
    public static final double BROWNOUT_RECOVERY_RATE_AMPS_PER_SECOND = 50.0;
    public static final double BROWNOUT_RECOVERY_WINDOW_SECONDS = 2.0;
    public static final double CURRENT_LIMIT_STEP_AMPS = 0.5;
    /** Initial plausibility threshold; tune from synchronized PDH/Talon logs before ACTIVE use. */
    public static final double MAXIMUM_CONTROLLED_CURRENT_MISMATCH_AMPS = 20.0;

    public static final double INVALID_ACTIVE_INPUT_GRACE_SECONDS = 0.1;

    public static final double DRIVE_STARTUP_LIMIT_AMPS = 50.0;
    public static final double ROLLER_STARTUP_LIMIT_AMPS = 50.0;
    public static final double EXTENDER_STARTUP_LIMIT_AMPS = 25.0;
    public static final double INDEXER_STARTUP_LIMIT_AMPS = 25.0;

    private EnergyConstants() {}

    public static AllocationConfig initialAllocationConfig() {
        return new AllocationConfig(
                new LoadPolicy(4, 5.0, 10.0, 25.0, DRIVE_STARTUP_LIMIT_AMPS),
                new LoadPolicy(2, 3.0, 10.0, 30.0, ROLLER_STARTUP_LIMIT_AMPS),
                new LoadPolicy(1, 3.0, 10.0, 20.0, EXTENDER_STARTUP_LIMIT_AMPS),
                new LoadPolicy(1, 3.0, 10.0, 20.0, INDEXER_STARTUP_LIMIT_AMPS),
                CURRENT_LIMIT_STEP_AMPS);
    }
}
