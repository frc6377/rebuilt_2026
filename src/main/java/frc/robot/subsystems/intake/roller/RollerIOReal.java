package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RollerIOReal implements RollerIO {

    private final TalonFX rollerMotor;

    private final LoggedNetworkNumber kRollerIntakePercent;
    private final LoggedNetworkNumber kRollerOuttakePercent;

    public RollerIOReal() {
        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(RollerConstants.MotorConfig.kInvertedReal)
                        .withNeutralMode(RollerConstants.MotorConfig.kNeutralMode))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(RollerConstants.MotorConfig.kRampPeriod))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(RollerConstants.MotorConfig.kStatorCurrentLimit));

        rollerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerMotorID);
        rollerMotor.getConfigurator().apply(config);

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
    public void setMode(NeutralModeValue mode) {
        rollerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
    }

    @Override
    public void setMotorPercentage(double percent) {
        rollerMotor.set(percent);
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
