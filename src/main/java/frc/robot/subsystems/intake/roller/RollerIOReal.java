package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;

public class RollerIOReal implements RollerIO {

    private final TalonFX rollerMotor;
    private final TalonFXConfiguration rollerMotorConfig;

    public RollerIOReal() {
        rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        rollerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        rollerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rollerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerMotorID);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
    }

    @Override
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
    }

    @Override
    public void start() {
        setRollerSpeed(IntakeConstants.RollerConstants.kIntakeSpeed);
    }

    @Override
    public void outtake() {
        setRollerSpeed(IntakeConstants.RollerConstants.kOuttakeSpeed);
    }

    @Override
    public void updateInputs(RollerIO.IntakeIOInputs inputs) {
        inputs.rollerSpeedPercentile = rollerMotor.get();
        inputs.rollerAppliedVolts = Volts.of(rollerMotor.getMotorVoltage().getValueAsDouble());
    }
}
