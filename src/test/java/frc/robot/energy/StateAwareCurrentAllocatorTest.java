package frc.robot.energy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.energy.StateAwareCurrentAllocator.Activity;
import frc.robot.energy.StateAwareCurrentAllocator.Allocation;
import frc.robot.energy.StateAwareCurrentAllocator.AllocationConfig;
import frc.robot.energy.StateAwareCurrentAllocator.LoadPolicy;
import frc.robot.energy.StateAwareCurrentAllocator.RollerActivity;
import org.junit.jupiter.api.Test;

class StateAwareCurrentAllocatorTest {
    private static final double EPSILON = 1.0e-9;

    @Test
    void inactiveMechanismsReceiveStandbyAndDriveReceivesTheRemainingCapacity() {
        StateAwareCurrentAllocator allocator = new StateAwareCurrentAllocator(testConfig());

        Allocation result = allocator.allocate(200.0, new Activity(false, false, false));

        assertEquals(30.0, result.drivePerMotor(), EPSILON);
        assertEquals(2.0, result.rollersPerMotor(), EPSILON);
        assertEquals(2.0, result.extenderPerMotor(), EPSILON);
        assertEquals(2.0, result.indexerPerMotor(), EPSILON);
        assertEquals(128.0, result.allocatedAmps(), EPSILON);
        assertEquals(72.0, result.unusedAmps(), EPSILON);
        assertEquals(0.0, result.activeMinimumDeficitAmps(), EPSILON);
        assertFalse(result.standbyDeficit());
    }

    @Test
    void activeMechanismsReachTargetsBeforeDriveReceivesItsMaximum() {
        StateAwareCurrentAllocator allocator = new StateAwareCurrentAllocator(testConfig());

        Allocation result = allocator.allocate(171.0, new Activity(true, true, true));

        assertEquals(30.0, result.drivePerMotor(), EPSILON);
        assertEquals(12.0, result.rollersPerMotor(), EPSILON);
        assertEquals(12.0, result.extenderPerMotor(), EPSILON);
        assertEquals(15.0, result.indexerPerMotor(), EPSILON);
        assertEquals(171.0, result.allocatedAmps(), EPSILON);
        assertEquals(0.0, result.unusedAmps(), EPSILON);
        assertEquals(0.0, result.activeMinimumDeficitAmps(), EPSILON);
        assertFalse(result.standbyDeficit());
    }

    @Test
    void idleRollersReceiveTheirMinimumButNotTheirActiveTarget() {
        StateAwareCurrentAllocator allocator = new StateAwareCurrentAllocator(testConfig());

        Allocation result = allocator.allocate(200.0, new Activity(RollerActivity.IDLE, false, false));

        assertEquals(30.0, result.drivePerMotor(), EPSILON);
        assertEquals(8.0, result.rollersPerMotor(), EPSILON);
        assertEquals(2.0, result.extenderPerMotor(), EPSILON);
        assertEquals(2.0, result.indexerPerMotor(), EPSILON);
    }

    @Test
    void severeDeficitProtectsIndexerExtenderAndRollersBeforeDrive() {
        StateAwareCurrentAllocator allocator = new StateAwareCurrentAllocator(testConfig());

        Allocation result = allocator.allocate(60.0, new Activity(true, true, true));

        assertEquals(6.5, result.drivePerMotor(), EPSILON);
        assertEquals(8.0, result.rollersPerMotor(), EPSILON);
        assertEquals(8.0, result.extenderPerMotor(), EPSILON);
        assertEquals(10.0, result.indexerPerMotor(), EPSILON);
        assertEquals(60.0, result.allocatedAmps(), EPSILON);
        assertEquals(0.0, result.unusedAmps(), EPSILON);
        assertEquals(14.0, result.activeMinimumDeficitAmps(), EPSILON);
        assertFalse(result.standbyDeficit());
    }

    @Test
    void activeIndexerOutranksInactiveMechanismStandbyInAnEmergency() {
        StateAwareCurrentAllocator allocator = new StateAwareCurrentAllocator(testConfig());

        Allocation result = allocator.allocate(10.0, new Activity(false, false, true));

        assertEquals(6.5, result.indexerPerMotor(), EPSILON);
        assertEquals(0.5, result.extenderPerMotor(), EPSILON);
        assertEquals(0.5, result.rollersPerMotor(), EPSILON);
        assertEquals(0.5, result.drivePerMotor(), EPSILON);
        assertEquals(10.0, result.allocatedAmps(), EPSILON);
        assertEquals(41.5, result.activeMinimumDeficitAmps(), EPSILON);
    }

    @Test
    void quantizesWholeMotorGroupsWithoutOversubscribingThePool() {
        StateAwareCurrentAllocator allocator = new StateAwareCurrentAllocator(testConfig());

        Allocation result = allocator.allocate(90.3, new Activity(true, true, true));

        assertEquals(10.0, result.drivePerMotor(), EPSILON);
        assertEquals(11.5, result.rollersPerMotor(), EPSILON);
        assertEquals(12.0, result.extenderPerMotor(), EPSILON);
        assertEquals(15.0, result.indexerPerMotor(), EPSILON);
        assertEquals(90.0, result.allocatedAmps(), EPSILON);
        assertEquals(0.3, result.unusedAmps(), EPSILON);
        assertTrue(result.allocatedAmps() <= 90.3);
    }

    @Test
    void reportsWhenPoolCannotCoverStandbyLimits() {
        StateAwareCurrentAllocator allocator = new StateAwareCurrentAllocator(testConfig());

        Allocation result = allocator.allocate(10.0, new Activity(false, false, false));

        assertTrue(result.standbyDeficit());
        assertTrue(result.allocatedAmps() <= 10.0);
        assertEquals(10.0, result.allocatedAmps() + result.unusedAmps(), EPSILON);
    }

    @Test
    void accountsForTheIrreducibleHardwareFloorWhenThePoolIsSmaller() {
        StateAwareCurrentAllocator allocator = new StateAwareCurrentAllocator(testConfig());

        Allocation result = allocator.allocate(0.0, new Activity(false, false, false));

        assertEquals(0.5, result.drivePerMotor(), EPSILON);
        assertEquals(0.5, result.rollersPerMotor(), EPSILON);
        assertEquals(0.5, result.extenderPerMotor(), EPSILON);
        assertEquals(0.5, result.indexerPerMotor(), EPSILON);
        assertEquals(4.0, result.allocatedAmps(), EPSILON);
        assertEquals(4.0, result.irreducibleFloorDeficitAmps(), EPSILON);
        assertEquals(0.0, result.unusedAmps(), EPSILON);
        assertTrue(result.standbyDeficit());
    }

    @Test
    void rejectsInvalidPoliciesAndConfiguration() {
        assertThrows(IllegalArgumentException.class, () -> new LoadPolicy(0, 2.0, 8.0, 12.0, 20.0));
        assertThrows(IllegalArgumentException.class, () -> new LoadPolicy(1, 9.0, 8.0, 12.0, 20.0));
        assertThrows(IllegalArgumentException.class, () -> new LoadPolicy(1, 2.0, 13.0, 12.0, 20.0));
        assertThrows(IllegalArgumentException.class, () -> new LoadPolicy(1, 2.0, 8.0, 21.0, 20.0));
        assertThrows(IllegalArgumentException.class, () -> new LoadPolicy(1, 2.0, 8.0, Double.NaN, 20.0));
        assertThrows(
                IllegalArgumentException.class,
                () -> new AllocationConfig(drivePolicy(), rollerPolicy(), extenderPolicy(), indexerPolicy(), 0.0));
    }

    @Test
    void rejectsInvalidPoolsAndNullActivity() {
        StateAwareCurrentAllocator allocator = new StateAwareCurrentAllocator(testConfig());

        assertThrows(IllegalArgumentException.class, () -> allocator.allocate(-1.0, new Activity(false, false, false)));
        assertThrows(
                IllegalArgumentException.class,
                () -> allocator.allocate(Double.NaN, new Activity(false, false, false)));
        assertThrows(NullPointerException.class, () -> allocator.allocate(100.0, null));
    }

    private static AllocationConfig testConfig() {
        return new AllocationConfig(drivePolicy(), rollerPolicy(), extenderPolicy(), indexerPolicy(), 0.5);
    }

    private static LoadPolicy drivePolicy() {
        return new LoadPolicy(4, 5.0, 10.0, 10.0, 30.0);
    }

    private static LoadPolicy rollerPolicy() {
        return new LoadPolicy(2, 2.0, 8.0, 12.0, 20.0);
    }

    private static LoadPolicy extenderPolicy() {
        return new LoadPolicy(1, 2.0, 8.0, 12.0, 15.0);
    }

    private static LoadPolicy indexerPolicy() {
        return new LoadPolicy(1, 2.0, 10.0, 15.0, 20.0);
    }
}
