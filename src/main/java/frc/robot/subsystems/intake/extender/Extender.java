package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.NerfModeController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Extender extends SubsystemBase {
    private final ExtenderIO io;
    private final ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();
    private boolean controllerActive = false;
    private double manualOutputPercent = 0.0;
    private long inputRevision = 0;
    private long closedLoopRequestInputRevision = Long.MIN_VALUE;
    private final NerfModeController nerfModeController;

    public Extender(ExtenderIO io, NerfModeController nerfModeController) {
        this.io = io;
        this.nerfModeController = nerfModeController;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputRevision++;
        io.periodic();
        Logger.processInputs("Intake/Extender", inputs);
        Logger.recordOutput(
                "Intake/Extender/CurrentCommand",
                getCurrentCommand() == null ? "null" : getCurrentCommand().toString());
    }

    public BooleanSupplier isAtTarget() {
        return io.atTarget();
    }

    public BooleanSupplier isExtended() {
        return io.isExtended();
    }

    public Command extendCommand() {
        return Commands.runOnce(this::extend, this).withName("ExtenderExtend");
    }

    public Command extendAndWaitCommand() {
        return Commands.run(this::extend, this).until(io.isExtended()).withName("ExtenderExtendAndWait");
    }

    public Command retractCommand() {
        return Commands.runOnce(this::retract, this).withName("ExtenderRetract");
    }

    public Command retractAndWaitCommand() {
        return Commands.runOnce(this::retract, this).until(io.isRetracted()).withName("ExtenderRetractAndWait");
    }

    public Command toggleCommand() {
        return Commands.runOnce(this::toggle, this).withName("ExtenderToggle");
    }

    public Command toggleSiftCommand() {
        return Commands.runOnce(this::toggleSift, this).withName("ExtenderToggleSift");
    }

    public Command goToSiftAngleOneCommand() {
        return Commands.runOnce(this::goToSiftAngleOne, this).withName("ExtenderSiftOne");
    }

    public Command goToSiftAngleTwoCommand() {
        return Commands.runOnce(this::goToSiftAngleTwo, this).withName("ExtenderSiftTwo");
    }

    public Command goToCustomAngleOneCommand() {
        return Commands.runOnce(this::goToCustomAngleOne, this).withName("ExtenderCustomOne");
    }

    public Command goToCustomAngleTwoCommand() {
        return Commands.runOnce(this::goToCustomAngleTwo, this).withName("ExtenderCustomTwo");
    }

    public Command zeroCommand() {
        return Commands.runOnce(this::zero, this).withName("ExtenderZero");
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this).withName("ExtenderStop");
    }

    public void setPidEnabled(boolean enabled) {
        controllerActive = enabled;
        if (enabled) {
            manualOutputPercent = 0.0;
            closedLoopRequestInputRevision = inputRevision;
        } else {
            closedLoopRequestInputRevision = Long.MIN_VALUE;
        }
        io.setPidEnabled(enabled);
    }

    public void setMode(NeutralModeValue mode) {
        io.setMode(mode);
    }

    public void setMotorPercentage(double percent) {
        controllerActive = false;
        manualOutputPercent = percent;
        closedLoopRequestInputRevision = Long.MIN_VALUE;
        io.setMotorPercentage(percent);
    }

    public void siftFuel() {
        toggleSift();
    }

    public double getSupplyCurrentAmps() {
        return inputs.motorSupplyCurrent.in(Amps);
    }

    public boolean isSupplyCurrentValid() {
        return inputs.motorSupplyCurrentValid;
    }

    public boolean isActiveForPowerManagement() {
        boolean freshClosedLoopRequest = controllerActive && closedLoopRequestInputRevision == inputRevision;
        boolean pidDemandingCurrent = controllerActive && (freshClosedLoopRequest || !inputs.atTarget);
        boolean measuredOutputActive = Math.abs(inputs.motorVoltage.in(Volts)) > 0.1;
        return pidDemandingCurrent || measuredOutputActive || Math.abs(manualOutputPercent) > 1e-3;
    }

    public void setSupplyCurrentLimit(double currentLimitAmps) {
        io.setSupplyCurrentLimit(currentLimitAmps);
    }

    private void prepareClosedLoop() {
        controllerActive = true;
        manualOutputPercent = 0.0;
        closedLoopRequestInputRevision = inputRevision;
    }

    private void extend() {
        prepareClosedLoop();
        io.extend();
    }

    private void retract() {
        prepareClosedLoop();
        io.retract();
    }

    private void toggle() {
        prepareClosedLoop();
        io.toggle();
    }

    private void toggleSift() {
        prepareClosedLoop();
        io.toggleSift();
    }

    private void goToSiftAngleOne() {
        prepareClosedLoop();
        io.goToSiftAngleOne();
    }

    private void goToSiftAngleTwo() {
        prepareClosedLoop();
        io.goToSiftAngleTwo();
    }

    private void goToCustomAngleOne() {
        prepareClosedLoop();
        io.goToCustomAngleOne();
    }

    private void goToCustomAngleTwo() {
        prepareClosedLoop();
        io.goToCustomAngleTwo();
    }

    private void zero() {
        controllerActive = false;
        manualOutputPercent = 0.0;
        closedLoopRequestInputRevision = Long.MIN_VALUE;
        io.zero();
    }

    private void stop() {
        controllerActive = false;
        manualOutputPercent = 0.0;
        closedLoopRequestInputRevision = Long.MIN_VALUE;
        io.stop();
    }
}
