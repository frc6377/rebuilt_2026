package frc.robot.energy;

import java.util.Objects;

/**
 * Allocates a controlled current pool to drive, intake rollers, the extender, and the indexer.
 *
 * <p>All outputs are per-motor limits. Whole physical groups are raised one current step at a time, so the returned
 * allocation never assumes that a fraction of a motor can consume a group budget. One current step per motor is an
 * irreducible hardware floor; if the pool is smaller, the returned allocation reports that unavoidable deficit.
 */
public final class StateAwareCurrentAllocator {
    private final AllocationConfig config;

    public StateAwareCurrentAllocator(AllocationConfig config) {
        this.config = Objects.requireNonNull(config);
    }

    public Allocation allocate(double controlledPoolAmps, Activity activity) {
        requireNonnegativeFinite(controlledPoolAmps, "controlledPoolAmps");
        Objects.requireNonNull(activity);

        GroupAllocation drive = new GroupAllocation(config.drive(), 1);
        GroupAllocation rollers = new GroupAllocation(config.rollers(), 1);
        GroupAllocation extender = new GroupAllocation(config.extender(), 1);
        GroupAllocation indexer = new GroupAllocation(config.indexer(), 1);
        double irreducibleFloorAmps = config.currentStepAmps()
                * (config.drive().motorCount()
                        + config.rollers().motorCount()
                        + config.extender().motorCount()
                        + config.indexer().motorCount());
        double irreducibleFloorDeficitAmps = Math.max(0.0, irreducibleFloorAmps - controlledPoolAmps);
        RemainingPool remainingPool = new RemainingPool(Math.max(0.0, controlledPoolAmps - irreducibleFloorAmps));

        // Active-safe minimums outrank standby headroom for inactive mechanisms.
        if (activity.indexerActive()) {
            raiseTo(indexer, config.indexer().activeMinimumPerMotor(), remainingPool);
        }
        if (activity.extenderActive()) {
            raiseTo(extender, config.extender().activeMinimumPerMotor(), remainingPool);
        }
        if (activity.rollersDemandingMinimum()) {
            raiseTo(rollers, config.rollers().activeMinimumPerMotor(), remainingPool);
        }
        // Drive has a standing drivability reserve, so it is always treated as active.
        raiseTo(drive, config.drive().activeMinimumPerMotor(), remainingPool);

        double activeMinimumDeficitAmps = deficitTo(drive, config.drive().activeMinimumPerMotor())
                + (activity.rollersDemandingMinimum()
                        ? deficitTo(rollers, config.rollers().activeMinimumPerMotor())
                        : 0.0)
                + (activity.extenderActive()
                        ? deficitTo(extender, config.extender().activeMinimumPerMotor())
                        : 0.0)
                + (activity.indexerActive()
                        ? deficitTo(indexer, config.indexer().activeMinimumPerMotor())
                        : 0.0);

        // Once active-safe minimums are covered, reserve standby headroom in mechanism priority order.
        raiseTo(indexer, config.indexer().standbyPerMotor(), remainingPool);
        raiseTo(extender, config.extender().standbyPerMotor(), remainingPool);
        raiseTo(rollers, config.rollers().standbyPerMotor(), remainingPool);
        raiseTo(drive, config.drive().standbyPerMotor(), remainingPool);

        boolean standbyDeficit = isBelow(drive, config.drive().standbyPerMotor())
                || isBelow(rollers, config.rollers().standbyPerMotor())
                || isBelow(extender, config.extender().standbyPerMotor())
                || isBelow(indexer, config.indexer().standbyPerMotor());

        if (activity.indexerActive()) {
            raiseTo(indexer, config.indexer().targetPerMotor(), remainingPool);
        }
        if (activity.extenderActive()) {
            raiseTo(extender, config.extender().targetPerMotor(), remainingPool);
        }
        if (activity.rollersActive()) {
            raiseTo(rollers, config.rollers().targetPerMotor(), remainingPool);
        }
        raiseTo(drive, config.drive().maximumPerMotor(), remainingPool);

        double drivePerMotor = currentPerMotor(drive);
        double rollersPerMotor = currentPerMotor(rollers);
        double extenderPerMotor = currentPerMotor(extender);
        double indexerPerMotor = currentPerMotor(indexer);
        double allocatedAmps = drivePerMotor * config.drive().motorCount()
                + rollersPerMotor * config.rollers().motorCount()
                + extenderPerMotor * config.extender().motorCount()
                + indexerPerMotor * config.indexer().motorCount();
        double unusedAmps = Math.max(0.0, controlledPoolAmps - allocatedAmps);

        return new Allocation(
                drivePerMotor,
                rollersPerMotor,
                extenderPerMotor,
                indexerPerMotor,
                allocatedAmps,
                unusedAmps,
                activeMinimumDeficitAmps,
                irreducibleFloorDeficitAmps,
                standbyDeficit);
    }

    private void raiseTo(GroupAllocation group, double targetPerMotor, RemainingPool remainingPool) {
        long targetSteps = wholeStepsAtOrBelow(targetPerMotor);
        long neededSteps = Math.max(0L, targetSteps - group.stepsPerMotor);
        if (neededSteps == 0L) {
            return;
        }

        double groupStepAmps = config.currentStepAmps() * group.policy.motorCount();
        long affordableSteps = (long) Math.floor(remainingPool.amps / groupStepAmps);
        while (affordableSteps > 0 && affordableSteps * groupStepAmps > remainingPool.amps) {
            affordableSteps--;
        }
        long allocatedSteps = Math.min(neededSteps, affordableSteps);
        group.stepsPerMotor += allocatedSteps;
        remainingPool.amps -= allocatedSteps * groupStepAmps;
    }

    private boolean isBelow(GroupAllocation group, double targetPerMotor) {
        return group.stepsPerMotor < wholeStepsAtOrBelow(targetPerMotor);
    }

    private double deficitTo(GroupAllocation group, double targetPerMotor) {
        long targetSteps = wholeStepsAtOrBelow(targetPerMotor);
        long missingSteps = Math.max(0L, targetSteps - group.stepsPerMotor);
        return missingSteps * config.currentStepAmps() * group.policy.motorCount();
    }

    private double currentPerMotor(GroupAllocation group) {
        return group.stepsPerMotor * config.currentStepAmps();
    }

    private long wholeStepsAtOrBelow(double currentAmps) {
        double quotient = currentAmps / config.currentStepAmps();
        double nearestInteger = Math.rint(quotient);
        if (Math.abs(quotient - nearestInteger) <= Math.max(1.0e-12, Math.ulp(quotient) * 4.0)) {
            return (long) nearestInteger;
        }
        return (long) Math.floor(quotient);
    }

    public record LoadPolicy(
            int motorCount,
            double standbyPerMotor,
            double activeMinimumPerMotor,
            double targetPerMotor,
            double maximumPerMotor) {
        public LoadPolicy {
            if (motorCount <= 0) {
                throw new IllegalArgumentException("motorCount must be positive");
            }
            requireNonnegativeFinite(standbyPerMotor, "standbyPerMotor");
            requireNonnegativeFinite(activeMinimumPerMotor, "activeMinimumPerMotor");
            requireNonnegativeFinite(targetPerMotor, "targetPerMotor");
            requireNonnegativeFinite(maximumPerMotor, "maximumPerMotor");
            if (standbyPerMotor > activeMinimumPerMotor
                    || activeMinimumPerMotor > targetPerMotor
                    || targetPerMotor > maximumPerMotor) {
                throw new IllegalArgumentException(
                        "current policy must satisfy standby <= active minimum <= target <= maximum");
            }
        }
    }

    public record AllocationConfig(
            LoadPolicy drive, LoadPolicy rollers, LoadPolicy extender, LoadPolicy indexer, double currentStepAmps) {
        public AllocationConfig {
            Objects.requireNonNull(drive);
            Objects.requireNonNull(rollers);
            Objects.requireNonNull(extender);
            Objects.requireNonNull(indexer);
            if (!Double.isFinite(currentStepAmps) || currentStepAmps <= 0.0) {
                throw new IllegalArgumentException("currentStepAmps must be finite and positive");
            }
            requireHardwareFloor(drive, currentStepAmps, "drive");
            requireHardwareFloor(rollers, currentStepAmps, "rollers");
            requireHardwareFloor(extender, currentStepAmps, "extender");
            requireHardwareFloor(indexer, currentStepAmps, "indexer");
        }
    }

    public enum RollerActivity {
        STOPPED,
        IDLE,
        ACTIVE
    }

    public record Activity(RollerActivity rollerActivity, boolean extenderActive, boolean indexerActive) {
        public Activity {
            Objects.requireNonNull(rollerActivity);
        }

        public Activity(boolean rollersActive, boolean extenderActive, boolean indexerActive) {
            this(rollersActive ? RollerActivity.ACTIVE : RollerActivity.STOPPED, extenderActive, indexerActive);
        }

        public boolean rollersActive() {
            return rollerActivity == RollerActivity.ACTIVE;
        }

        public boolean rollersIdle() {
            return rollerActivity == RollerActivity.IDLE;
        }

        private boolean rollersDemandingMinimum() {
            return rollerActivity != RollerActivity.STOPPED;
        }
    }

    public record Allocation(
            double drivePerMotor,
            double rollersPerMotor,
            double extenderPerMotor,
            double indexerPerMotor,
            double allocatedAmps,
            double unusedAmps,
            double activeMinimumDeficitAmps,
            double irreducibleFloorDeficitAmps,
            boolean standbyDeficit) {}

    private static final class GroupAllocation {
        private final LoadPolicy policy;
        private long stepsPerMotor;

        private GroupAllocation(LoadPolicy policy, long initialStepsPerMotor) {
            this.policy = policy;
            this.stepsPerMotor = initialStepsPerMotor;
        }
    }

    private static final class RemainingPool {
        private double amps;

        private RemainingPool(double amps) {
            this.amps = amps;
        }
    }

    private static void requireNonnegativeFinite(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and nonnegative");
        }
    }

    private static void requireHardwareFloor(LoadPolicy policy, double currentStepAmps, String name) {
        if (policy.maximumPerMotor() < currentStepAmps) {
            throw new IllegalArgumentException(name + " maximum must cover one hardware current step");
        }
    }
}
