package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Extender extends SubsystemBase {
    private final ExtenderIO io;
    private final ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

    public Extender(ExtenderIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.periodic();
        Logger.processInputs("Intake/Extender", inputs);
        Logger.recordOutput(
                "Intake/Extender/CurrentCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    public boolean isExtended() {
        return io.isExtended().getAsBoolean();
    }

    public Command extendCommand() {
        return Commands.runOnce(io::extend, this).withName("ExtenderExtend");
    }

    public Command extendAndWaitCommand() {
        return Commands.runOnce(io::extend, this).until(io.isExtended()).withName("ExtenderExtendAndWait");
    }

    public Command retractCommand() {
        return Commands.runOnce(io::retract, this).withName("ExtenderRetract");
    }

    public Command retractAndWaitCommand() {
        return Commands.runOnce(io::retract, this).until(io.isRetracted()).withName("ExtenderRetractAndWait");
    }

    public Command toggleCommand() {
        return Commands.runOnce(io::toggle, this).withName("ExtenderToggle");
    }

    public Command goToSiftAngleOneCommand() {
        return Commands.runOnce(io::goToSiftAngleOne, this).withName("ExtenderSiftOne");
    }

    public Command goToSiftAngleTwoCommand() {
        return Commands.runOnce(io::goToSiftAngleTwo, this).withName("ExtenderSiftTwo");
    }

    public Command goToCustomAngleOneCommand() {
        return Commands.runOnce(io::goToCustomAngleOne, this).withName("ExtenderCustomOne");
    }

    public Command goToCustomAngleTwoCommand() {
        return Commands.runOnce(io::goToCustomAngleTwo, this).withName("ExtenderCustomTwo");
    }

    public Command zeroCommand() {
        return Commands.runOnce(io::zero, this).withName("ExtenderZero");
    }

    public Command stopCommand() {
        return Commands.runOnce(io::stop, this).withName("ExtenderStop");
    }

    public void setPidEnabled(boolean enabled) {
        io.setPidEnabled(enabled);
    }

    public void setMode(NeutralModeValue mode) {
        io.setMode(mode);
    }

    public void setMotorPercentage(double percent) {
        io.setMotorPercentage(percent);
    }

    public Command siftFuelPositionCommand() {
        return Commands.repeatingSequence(
                        Commands.run(io::goToSiftAngleOne, this)
                                .until(() -> io.atTarget().getAsBoolean() || io.getCurrent().gte(Amps.of(15)))
                                
                                .withTimeout(1.5),
                        Commands.run(io::goToSiftAngleTwo, this)
                                .until(() -> io.atTarget().getAsBoolean() || io.getCurrent().gte(Amps.of(15)))
                                
                                .withTimeout(1.5))
                .withName("ExtenderSiftPositionFuel");
    }

    public Command siftFuelSpeedCommand() {
        return runUntilCurrentCommand(-0.1, Amps.of(8))
                .andThen(Commands.repeatingSequence(
                        runUntilCurrentCommand(0.25, Amps.of(8)).withTimeout(1),
                        runUntilCurrentCommand(-0.25, Amps.of(8)).withTimeout(1)));
    }

    public Command runUntilCurrentCommand(double percent, Current current) {
        return Commands.run(() -> io.setMotorPercentage(percent))
                .until(() -> io.getCurrent().gte(current) )
                .withName("ExtenderRunUntilCurrent");
    }
}
