package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import frc.robot.util.TunableTalonFX;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class PIDRollerIOReal implements RollerIO {

    private final @NotNull TunableTalonFX leaderMotor;
    private final @Nullable TalonFX followerMotor;
    private final @NotNull LoggedNetworkNumber intakeSpeed;
    private final @NotNull LoggedNetworkNumber outtakeSpeed;
    private final @NotNull LoggedNetworkNumber idleSpeed;

    public PIDRollerIOReal() {
        @NotNull Slot0Configs rollerPID = new Slot0Configs();
        rollerPID.kP = IntakeConstants.RollerConstants.PIDF.kP;
        rollerPID.kI = IntakeConstants.RollerConstants.PIDF.kI;
        rollerPID.kD = IntakeConstants.RollerConstants.PIDF.kD;
        this.intakeSpeed = new LoggedNetworkNumber(
                "Intake/Roller/IntakeSpeed", IntakeConstants.RollerConstants.kIntakeSpeed.in(RPM));
        this.outtakeSpeed = new LoggedNetworkNumber(
                "Intake/Roller/OuttakeSpeed", IntakeConstants.RollerConstants.kOuttakeSpeed.in(RPM));
        this.idleSpeed =
                new LoggedNetworkNumber("Intake/Roller/IdleSpeed", IntakeConstants.RollerConstants.kIdleSpeed.in(RPM));

        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(RollerConstants.MotorConfig.kInverted)
                        .withInverted(RollerConstants.MotorConfig.kInverted)
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

        this.intakeSpeed.set(RollerConstants.kIntakeSpeed.in(RPM));
        this.outtakeSpeed.set(RollerConstants.kOuttakeSpeed.in(RPM));
        this.idleSpeed.set(RollerConstants.kIdleSpeed.in(RPM));

        this.leaderMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kRollerLeaderMotorID, "rio", "Intake/RollerPID", rollerPID);
        this.leaderMotor.applyConfiguration(config);

        if (RollerConstants.kfollowerEnabled) {
            this.followerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerFollowerMotorID);
            this.followerMotor.getConfigurator().apply(config);
        } else {
            this.followerMotor = null;
        }
    }

    public void setFollower() {
        if (null != followerMotor) {
            this.followerMotor.setControl(
                    new Follower(this.leaderMotor.getDeviceID(), RollerConstants.MotorConfig.kFollowerInverted));
        }
    }

    public void setRollerSpeed(@NotNull AngularVelocity speed) {
        this.leaderMotor.setControl(new VelocityVoltage(speed));
        this.setFollower();
    }

    @Override
    public void setRollerVoltage(@NotNull Voltage volts) {
        this.leaderMotor.setControl(new VoltageOut(volts));
        this.setFollower();
    }

    @Override
    public void idle() {
        if (RollerConstants.kIdleEnabled) {
            this.setRollerSpeed(RPM.of(this.idleSpeed.get()));
        }
    }

    @Override
    public boolean isRunning() {
        return 0.1 < Math.abs(leaderMotor.get());
    }

    @Override
    public void stop() {
        this.setRollerSpeed(RPM.zero());
        this.setRollerSpeed(RPM.zero());
    }

    @Override
    public void start() {
        this.setRollerSpeed(RPM.of(this.intakeSpeed.get()));
        this.setRollerSpeed(RPM.of(this.intakeSpeed.get()));
        this.setRollerSpeed(RPM.of(this.intakeSpeed.get()));
    }

    @Override
    public void outtake() {
        this.setRollerSpeed(RPM.of(this.outtakeSpeed.get()));
        this.setRollerSpeed(RPM.of(this.outtakeSpeed.get()));
        this.setRollerSpeed(RPM.of(this.outtakeSpeed.get()));
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

    @Override
    public void periodic() {
        this.leaderMotor.updateTunableGains();
        this.leaderMotor.updateTunableGains();
    }
}
