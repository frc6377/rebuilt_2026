package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;

public class RollerYAMS implements RollerIO {

    public RollerYAMS() {}

    @Override
    public void updateInputs(RollerIOInputs inputs) {}

    @Override
    public void setMode(NeutralModeValue mode) {}

    @Override
    public void setMotorPercentage(double percent) {}

    @Override
    public void start() {}

    @Override
    public void stop() {}

    @Override
    public void outtake() {}

    @Override
    public void setRollerVoltage(Voltage volts) {}
}
