package frc.robot.subsystems.intake.extender;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Extender extends SubsystemBase {
    private final ExtenderIO io;
    private final ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

    public Extender(ExtenderIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        this.io.periodic();
        Logger.processInputs("Intake/Extender", this.inputs);
        Logger.recordOutput(
                "Intake/Extender/CurrentCommand",
                null == this.getCurrentCommand() ? "null" : this.getCurrentCommand().toString());
    }

    public BooleanSupplier isAtTarget() {
        return this.io.atTarget();
    }

    public BooleanSupplier isExtended() {
        return this.io.isExtended();
    }

    public Command extendCommand() {
        return Commands.runOnce(this.io::extend, this).withName("ExtenderExtend");
    }

    public Command extendAndWaitCommand() {
        return Commands.run(this.io::extend, this).until(this.io.isExtended()).withName("ExtenderExtendAndWait");
    }

    public Command retractCommand() {
        return Commands.runOnce(this.io::retract, this).withName("ExtenderRetract");
    }

    public Command retractAndWaitCommand() {
        return Commands.runOnce(this.io::retract, this).until(this.io.isRetracted()).withName("ExtenderRetractAndWait");
    }

    public Command toggleCommand() {
        return Commands.runOnce(this.io::toggle, this).withName("ExtenderToggle");
    }

    public Command toggleSiftCommand() {
        return Commands.runOnce(this.io::toggleSift, this).withName("ExtenderToggleSift");
    }

    public Command goToSiftAngleOneCommand() {
        return Commands.runOnce(this.io::goToSiftAngleOne, this).withName("ExtenderSiftOne");
    }

    public Command goToSiftAngleTwoCommand() {
        return Commands.runOnce(this.io::goToSiftAngleTwo, this).withName("ExtenderSiftTwo");
    }

    public Command goToCustomAngleOneCommand() {
        return Commands.runOnce(this.io::goToCustomAngleOne, this).withName("ExtenderCustomOne");
    }

    public Command goToCustomAngleTwoCommand() {
        return Commands.runOnce(this.io::goToCustomAngleTwo, this).withName("ExtenderCustomTwo");
    }

    public Command zeroCommand() {
        return Commands.runOnce(this.io::zero, this).withName("ExtenderZero");
    }

    public Command stopCommand() {
        return Commands.runOnce(this.io::stop, this).withName("ExtenderStop");
    }

    public void setPidEnabled(boolean enabled) {
        this.io.setPidEnabled(enabled);
    }

    public void setMode(NeutralModeValue mode) {
        this.io.setMode(mode);
    }

    public void setMotorPercentage(double percent) {
        this.io.setMotorPercentage(percent);
    }

    public void siftFuel() {
        this.io.toggleSift();
    }
}
