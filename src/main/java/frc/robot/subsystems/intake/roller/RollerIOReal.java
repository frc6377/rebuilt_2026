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

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RollerIOReal implements RollerIO {

    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    private final LoggedNetworkNumber kRollerIntakePercent;
    private final LoggedNetworkNumber kRollerOuttakePercent;
    private final LoggedNetworkNumber kRollerIdlePercent;

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
        kRollerIdlePercent = new LoggedNetworkNumber("Intake/Roller/IdlePercent", RollerConstants.kIdlePercent);
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
            setRollerSpeed(kRollerIdlePercent.get());
        }
    }

    @Override
    public boolean isRunning() {
        return Math.abs(leaderMotor.get()) > 0.1;
    }

    @Override
    public void stop() {
        setRollerSpeed(0.0);
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
        inputs.leaderSpeedPercentile = leaderMotor.get();
        inputs.leaderAppliedVolts = leaderMotor.getMotorVoltage().getValue();
        inputs.leaderVelocity = leaderMotor.getVelocity().getValue();
        inputs.leaderStatorCurrent = leaderMotor.getStatorCurrent().getValue();
        inputs.leaderMotorTemp = leaderMotor.getDeviceTemp().getValue();

        if (followerMotor != null) {
            inputs.followerSpeedPercentile = followerMotor.get();
            inputs.followerAppliedVolts = followerMotor.getMotorVoltage().getValue();
            inputs.followerVelocity = followerMotor.getVelocity().getValue();
            inputs.followerStatorCurrent = followerMotor.getStatorCurrent().getValue();
            inputs.followerMotorTemp = followerMotor.getDeviceTemp().getValue();
        }

        inputs.isRunning = isRunning();
    }
}
