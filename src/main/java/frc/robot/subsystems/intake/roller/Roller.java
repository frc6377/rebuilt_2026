package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
    private final SysIdRoutine sysIdRoutine;
    private int loopCounter = 0;

    public Roller(RollerIO io) {
        this.io = io;
        setDefaultCommand(run(io::idle).withName("RollerIdle"));
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                        null, // Use default timeout (10 s)
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(io::setRollerVoltage, null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.periodic();
        loopCounter++;
        if (Constants.currentMode != Constants.Mode.SIM || loopCounter % 2 == 0) {
            Logger.processInputs("Intake/Roller", inputs);
            Logger.recordOutput(
                    "Intake/Roller/CurrentCommand",
                    getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command runCommand() {
        return Commands.runEnd(io::start, io::stop, this).withName("RollerRun");
    }

    public Command outtakeCommand() {
        return Commands.runEnd(io::outtake, io::stop, this).withName("RollerOuttake");
    }

    public Command stopCommand() {
        return Commands.runOnce(io::stop, this).withName("RollerStop");
    }

    public boolean isRunning() {
        return Math.abs(inputs.leaderSpeedPercentile) > 0.1;
    }

    public int getIntakedFuel() {
        return io.getIntakedFuel();
    }

    public void setMode(NeutralModeValue mode) {
        io.setMode(mode);
    }

    public void setMotorPercentage(double percent) {
        io.setMotorPercentage(percent);
    }
}
