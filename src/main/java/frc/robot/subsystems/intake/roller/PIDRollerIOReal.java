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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class PIDRollerIOReal implements RollerIO {

    private final TunableTalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final Slot0Configs rollerPID;
    private final LoggedNetworkNumber intakeSpeed;
    private final LoggedNetworkNumber outtakeSpeed;
    private final LoggedNetworkNumber idleSpeed;

    public PIDRollerIOReal() {
        rollerPID = new Slot0Configs();
        rollerPID.kP = IntakeConstants.RollerConstants.PIDF.kP;
        rollerPID.kI = IntakeConstants.RollerConstants.PIDF.kI;
        rollerPID.kD = IntakeConstants.RollerConstants.PIDF.kD;
        intakeSpeed = new LoggedNetworkNumber(
                "Intake/Roller/IntakeSpeed", IntakeConstants.RollerConstants.kIntakeSpeed.in(RPM));
        outtakeSpeed = new LoggedNetworkNumber(
                "Intake/Roller/OuttakeSpeed", IntakeConstants.RollerConstants.kOuttakeSpeed.in(RPM));
        idleSpeed =
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

        intakeSpeed.set(RollerConstants.kIntakeSpeed.in(RPM));
        outtakeSpeed.set(RollerConstants.kOuttakeSpeed.in(RPM));
        idleSpeed.set(RollerConstants.kIdleSpeed.in(RPM));

        leaderMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kRollerLeaderMotorID, "rio", "Intake/RollerPID", rollerPID);
        leaderMotor.applyConfiguration(config);

        if (RollerConstants.kfollowerEnabled) {
            followerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerFollowerMotorID);
            followerMotor.getConfigurator().apply(config);
        } else {
            followerMotor = null;
        }
    }

    public void setFollower() {
        if (followerMotor != null) {
            followerMotor.setControl(
                    new Follower(leaderMotor.getDeviceID(), RollerConstants.MotorConfig.kFollowerInverted));
        }
    }

    public void setRollerSpeed(AngularVelocity speed) {
        leaderMotor.setControl(new VelocityVoltage(speed));
        setFollower();
    }

    @Override
    public void setRollerVoltage(Voltage volts) {
        leaderMotor.setControl(new VoltageOut(volts));
        setFollower();
    }

    @Override
    public void idle() {
        if (RollerConstants.kIdleEnabled) {
            setRollerSpeed(RPM.of(idleSpeed.get()));
        }
    }

    @Override
    public boolean isRunning() {
        return Math.abs(leaderMotor.get()) > 0.1;
    }

    @Override
    public void stop() {
        setRollerSpeed(RPM.zero());
        setRollerSpeed(RPM.zero());
    }

    @Override
    public void start() {
        setRollerSpeed(RPM.of(intakeSpeed.get()));
        setRollerSpeed(RPM.of(intakeSpeed.get()));
        setRollerSpeed(RPM.of(intakeSpeed.get()));
    }

    @Override
    public void outtake() {
        setRollerSpeed(RPM.of(outtakeSpeed.get()));
        setRollerSpeed(RPM.of(outtakeSpeed.get()));
        setRollerSpeed(RPM.of(outtakeSpeed.get()));
    }

    @Override
    public int getIntakedFuel() {
        return 0;
    }

    @Override
    public void setMode(NeutralModeValue mode) {
        leaderMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
        leaderMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
    }

    @Override
    public void setMotorPercentage(double percent) {
        leaderMotor.set(percent);
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

    @Override
    public void periodic() {
        leaderMotor.updateTunableGains();
        leaderMotor.updateTunableGains();
    }
}
