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
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RollerIOReal implements RollerIO {

    private final @NotNull TalonFX leaderMotor;
    private final @NotNull TalonFX followerMotor;

    private final @NotNull LoggedNetworkNumber kRollerIntakePercent;
    private final @NotNull LoggedNetworkNumber kRollerOuttakePercent;
    private final @NotNull LoggedNetworkNumber kRollerIdlePercent;

    public RollerIOReal() {
        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(RollerConstants.MotorConfig.kInverted)
                        .withInverted(RollerConstants.MotorConfig.kInverted)
                        .withNeutralMode(RollerConstants.MotorConfig.kNeutralMode))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(RollerConstants.MotorConfig.kRampPeriod))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(RollerConstants.MotorConfig.kStatorCurrentLimit));

        this.leaderMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerLeaderMotorID);
        this.leaderMotor.getConfigurator().apply(config);

        this.followerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerFollowerMotorID);
        this.followerMotor.getConfigurator().apply(config);

        this.kRollerIntakePercent = new LoggedNetworkNumber("Intake/Roller/IntakePercent", RollerConstants.kIntakePercent);
        this.kRollerOuttakePercent =
                new LoggedNetworkNumber("Intake/Roller/OuttakePercent", RollerConstants.kOuttakePercent);
        this.kRollerIdlePercent = new LoggedNetworkNumber("Intake/Roller/IdlePercent", RollerConstants.kIdlePercent);
    }

    public void setRollerSpeed(double speed) {
        this.leaderMotor.set(speed);
        this.setFollower();
    }

    public void setFollower() {
        if (null != followerMotor) {
            this.followerMotor.setControl(
                    new Follower(this.leaderMotor.getDeviceID(), RollerConstants.MotorConfig.kFollowerInverted));
        }
    }

    @Override
    public void idle() {
        if (RollerConstants.kIdleEnabled) {
            this.setRollerSpeed(this.kRollerIdlePercent.get());
        }
    }

    @Override
    public boolean isRunning() {
        return 0.1 < Math.abs(leaderMotor.get());
    }

    @Override
    public void stop() {
        this.setRollerSpeed(0.0);
    }

    @Override
    public void start() {
        this.setRollerSpeed(this.kRollerIntakePercent.get());
    }

    @Override
    public void outtake() {
        this.setRollerSpeed(this.kRollerOuttakePercent.get());
    }

    @Override
    public int getIntakedFuel() {
        return 0;
    }

    @Override
    public void setMode(NeutralModeValue mode) {
        this.leaderMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
        this.leaderMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
    }

    @Override
    public void setMotorPercentage(double percent) {
        this.leaderMotor.set(percent);
        this.leaderMotor.set(percent);
    }

    @Override
    public void updateInputs(RollerIO.@NotNull RollerIOInputs inputs) {
        inputs.leaderSpeedPercentile = this.leaderMotor.get();
        inputs.leaderAppliedVolts = this.leaderMotor.getMotorVoltage().getValue();
        inputs.leaderVelocity = this.leaderMotor.getVelocity().getValue();
        inputs.leaderStatorCurrent = this.leaderMotor.getStatorCurrent().getValue();
        inputs.leaderMotorTemp = this.leaderMotor.getDeviceTemp().getValue();

        if (null != followerMotor) {
            inputs.followerSpeedPercentile = this.followerMotor.get();
            inputs.followerAppliedVolts = this.followerMotor.getMotorVoltage().getValue();
            inputs.followerVelocity = this.followerMotor.getVelocity().getValue();
            inputs.followerStatorCurrent = this.followerMotor.getStatorCurrent().getValue();
            inputs.followerMotorTemp = this.followerMotor.getDeviceTemp().getValue();
        }

        inputs.isRunning = this.isRunning();
        inputs.leaderSpeedPercentile = this.leaderMotor.get();
        inputs.leaderAppliedVolts = this.leaderMotor.getMotorVoltage().getValue();
        inputs.leaderVelocity = this.leaderMotor.getVelocity().getValue();
        inputs.leaderStatorCurrent = this.leaderMotor.getStatorCurrent().getValue();
        inputs.leaderMotorTemp = this.leaderMotor.getDeviceTemp().getValue();

        if (null != followerMotor) {
            inputs.followerSpeedPercentile = this.followerMotor.get();
            inputs.followerAppliedVolts = this.followerMotor.getMotorVoltage().getValue();
            inputs.followerVelocity = this.followerMotor.getVelocity().getValue();
            inputs.followerStatorCurrent = this.followerMotor.getStatorCurrent().getValue();
            inputs.followerMotorTemp = this.followerMotor.getDeviceTemp().getValue();
        }

        inputs.isRunning = this.isRunning();
    }
}
