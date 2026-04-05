package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Volts;


import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();


    public Roller(RollerIO io) {
        this.io = io;
        setDefaultCommand(run(io::idle).withName("RollerIdle"));
}

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.periodic();
        Logger.processInputs("Intake/Roller", inputs);
        Logger.recordOutput(
                "Intake/Roller/CurrentCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
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
