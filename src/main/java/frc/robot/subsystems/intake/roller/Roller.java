package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
    private final @NotNull SysIdRoutine sysIdRoutine;

    public Roller(@NotNull RollerIO io) {
        this.io = io;
        this.setDefaultCommand(this.run(io::idle).withName("RollerIdle"));
        this.sysIdRoutine = new SysIdRoutine(
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
        this.io.updateInputs(this.inputs);
        this.io.periodic();
        Logger.processInputs("Intake/Roller", this.inputs);
        Logger.recordOutput(
                "Intake/Roller/CurrentCommand",
                null != getCurrentCommand() ? this.getCurrentCommand().getName() : "None");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.dynamic(direction);
    }

    public Command runCommand() {
        return Commands.runEnd(this.io::start, this.io::stop, this).withName("RollerRun");
    }

    public Command outtakeCommand() {
        return Commands.runEnd(this.io::outtake, this.io::stop, this).withName("RollerOuttake");
    }

    public Command stopCommand() {
        return Commands.runOnce(this.io::stop, this).withName("RollerStop");
    }

    public boolean isRunning() {
        return 0.1 < Math.abs(inputs.leaderSpeedPercentile);
    }

    public int getIntakedFuel() {
        return this.io.getIntakedFuel();
    }

    public void setMode(NeutralModeValue mode) {
        this.io.setMode(mode);
    }

    public void setMotorPercentage(double percent) {
        this.io.setMotorPercentage(percent);
    }
}
