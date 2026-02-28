package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import frc.robot.util.TunableTalonFX;

public class PIDRollerIOReal implements RollerIO {

    private final TunableTalonFX rollerMotor;
    private final TalonFXConfiguration rollerMotorConfig;
    private final Slot0Configs rollerPID;
    private final CurrentLimitsConfigs currentConfig;

    public PIDRollerIOReal() {

        currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = RollerConstants.MotorConfig.kStatorCurrentLimit.in(Amps);

        rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = RollerConstants.MotorConfig.kRampPeriod;
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerPID = new Slot0Configs();
        rollerPID.kP = IntakeConstants.RollerConstants.PIDF.kP;
        rollerPID.kI = IntakeConstants.RollerConstants.PIDF.kI;
        rollerPID.kD = IntakeConstants.RollerConstants.PIDF.kD;

        rollerMotor =
                new TunableTalonFX(Constants.CANIDs.MotorIDs.kRollerMotorID, "rio", "Intake/RollerPID", rollerPID);
        rollerMotor.applyConfiguration(rollerMotorConfig);
        rollerMotor.getConfigurator().apply(currentConfig);
    }

    public void setRollerSpeed(AngularVelocity speed) {
        rollerMotor.setControl(new VelocityVoltage(speed));
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
        ;
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

    @Override
    public void periodic() {
        rollerMotor.updateTunableGains();
    }
}
