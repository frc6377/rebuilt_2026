package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import frc.robot.util.TunableTalonFX;

public class PIDRollerIOReal implements RollerIO {

    private final TunableTalonFX rollerMotor;
    private final Slot0Configs rollerPID;

    public PIDRollerIOReal() {
        rollerPID = new Slot0Configs();
        rollerPID.kP = IntakeConstants.RollerConstants.PIDF.kP;
        rollerPID.kI = IntakeConstants.RollerConstants.PIDF.kI;
        rollerPID.kD = IntakeConstants.RollerConstants.PIDF.kD;

        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(RollerConstants.MotorConfig.kInvertedReal)
                        .withNeutralMode(RollerConstants.MotorConfig.kNeutralMode))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(RollerConstants.MotorConfig.kRampPeriod))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(RollerConstants.MotorConfig.kStatorCurrentLimit))
                .withSlot0(new Slot0Configs()
                        .withKP(RollerConstants.PIDF.kP)
                        .withKI(RollerConstants.PIDF.kI)
                        .withKD(RollerConstants.PIDF.kD)
                        .withKS(RollerConstants.PIDF.kS)
                        .withKV(RollerConstants.PIDF.kV)
                        .withKA(RollerConstants.PIDF.kA));

        rollerMotor =
                new TunableTalonFX(Constants.CANIDs.MotorIDs.kRollerMotorID, "rio", "Intake/RollerPID", rollerPID);
        rollerMotor.applyConfiguration(config);
    }

    public void setRollerSpeed(AngularVelocity speed) {
        rollerMotor.setControl(new VelocityVoltage(speed));
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

    @Override
    public void periodic() {
        rollerMotor.updateTunableGains();
    }
}
