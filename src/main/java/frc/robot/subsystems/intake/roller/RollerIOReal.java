package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RollerIOReal implements RollerIO {

    private final TalonFX rollerMotor;
    private final TalonFXConfiguration rollerMotorConfig;
    private final CurrentLimitsConfigs currentConfig;

    private final LoggedNetworkNumber kRollerIntakePercent;
    private final LoggedNetworkNumber kRollerOuttakePercent;

    public RollerIOReal() {
        currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = RollerConstants.MotorConfig.kStatorCurrentLimit.in(Amps);

        rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = RollerConstants.MotorConfig.kRampPeriod;
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerMotorID);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
        rollerMotor.getConfigurator().apply(currentConfig);

        kRollerIntakePercent = new LoggedNetworkNumber("Intake/Roller/IntakePercent", RollerConstants.kIntakePercent);
        kRollerOuttakePercent =
                new LoggedNetworkNumber("Intake/Roller/OuttakePercent", RollerConstants.kOuttakePercent);
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
    }

    @Override
    public void start() {
        setRollerSpeed(kRollerIntakePercent.get());
    }

    @Override
    public void outtake() {
        setRollerSpeed(kRollerOuttakePercent.get());
    }

    @Override
    public int getIntakedFuel() {
        return 0;
    }

    @Override
    public void updateInputs(RollerIO.RollerIOInputs inputs) {
        inputs.rollerSpeedPercentile = rollerMotor.get();
        inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValue();
        inputs.rollerVelocity = rollerMotor.getVelocity().getValue();
        inputs.statorCurrent = rollerMotor.getStatorCurrent().getValue();
        inputs.motorTemp = rollerMotor.getDeviceTemp().getValue();
    }
}
