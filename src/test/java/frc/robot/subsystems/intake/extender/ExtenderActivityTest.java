package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ExtenderActivityTest {
    @Test
    void aNewClosedLoopRequestIsActiveBeforeFreshIoInputsArrive() {
        Extender extender = new Extender(new AtTargetExtenderIO());
        extender.periodic();
        assertFalse(extender.isActiveForPowerManagement());

        extender.extendCommand().initialize();
        assertTrue(extender.isActiveForPowerManagement());

        extender.periodic();
        assertFalse(extender.isActiveForPowerManagement());
    }

    @Test
    void measuredMotorOutputRemainsActiveEvenWhenThePidFlagIsClear() {
        Extender extender = new Extender(new EnergizedExtenderIO());

        extender.periodic();

        assertTrue(extender.isActiveForPowerManagement());
    }

    private static final class AtTargetExtenderIO implements ExtenderIO {
        @Override
        public void updateInputs(ExtenderIOInputs inputs) {
            inputs.atTarget = true;
            inputs.motorVoltage = Volts.zero();
        }
    }

    private static final class EnergizedExtenderIO implements ExtenderIO {
        @Override
        public void updateInputs(ExtenderIOInputs inputs) {
            inputs.atTarget = true;
            inputs.motorVoltage = Volts.of(2.0);
        }
    }
}
