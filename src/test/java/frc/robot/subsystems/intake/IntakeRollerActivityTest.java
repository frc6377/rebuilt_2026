package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.energy.StateAwareCurrentAllocator.RollerActivity;
import org.junit.jupiter.api.Test;

class IntakeRollerActivityTest {
    @Test
    void distinguishesStoppedIdleAndActiveCommandedSpeeds() {
        assertEquals(RollerActivity.STOPPED, Intake.classifyRollerActivity(RPM.zero()));
        assertEquals(RollerActivity.IDLE, Intake.classifyRollerActivity(RPM.of(100.0)));
        assertEquals(RollerActivity.IDLE, Intake.classifyRollerActivity(RPM.of(-100.0)));
        assertEquals(RollerActivity.ACTIVE, Intake.classifyRollerActivity(RPM.of(2500.0)));
        assertEquals(RollerActivity.ACTIVE, Intake.classifyRollerActivity(RPM.of(-1800.0)));
    }
}
