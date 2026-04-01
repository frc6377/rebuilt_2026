package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RollerIOReal implements RollerIO {

    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    private final LoggedNetworkNumber kRollerIntakePercent;
    private final LoggedNetworkNumber kRollerOuttakePercent;

    public RollerIOReal() {
        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(RollerConstants.MotorConfig.kInverted)
                        .withNeutralMode(RollerConstants.MotorConfig.kNeutralMode))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(RollerConstants.MotorConfig.kRampPeriod))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(RollerConstants.MotorConfig.kStatorCurrentLimit));

        leaderMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerLeaderMotorID);
        leaderMotor.getConfigurator().apply(config);

        followerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerFollowerMotorID);
        followerMotor.getConfigurator().apply(config);

        kRollerIntakePercent = new LoggedNetworkNumber("Intake/Roller/IntakePercent", RollerConstants.kIntakePercent);
        kRollerOuttakePercent =
                new LoggedNetworkNumber("Intake/Roller/OuttakePercent", RollerConstants.kOuttakePercent);
    }

    public void setRollerSpeed(double speed) {
        leaderMotor.set(speed);
        setFollower();
    }

    public void setFollower() {
        if (followerMotor != null) {
            followerMotor.setControl(
                    new Follower(leaderMotor.getDeviceID(), RollerConstants.MotorConfig.kFollowerInverted));
        }
    }

    @Override
    public void idle() {
        if (RollerConstants.kIdleEnabled) {
            setRollerSpeed(RollerConstants.kIdlePercent);
        }
    }

    @Override
    public void stop() {
        leaderMotor.stopMotor();
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
        leaderMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
    }

    @Override
    public void setMotorPercentage(double percent) {
        leaderMotor.set(percent);
    }

    @Override
    public void updateInputs(RollerIO.RollerIOInputs inputs) {
        inputs.rollerSpeedPercentile = leaderMotor.get();
        inputs.rollerAppliedVolts = leaderMotor.getMotorVoltage().getValue();
        inputs.rollerVelocity = leaderMotor.getVelocity().getValue();
        inputs.statorCurrent = leaderMotor.getStatorCurrent().getValue();
        inputs.motorTemp = leaderMotor.getDeviceTemp().getValue();
    }
}
