package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import frc.robot.util.TunableTalonFX;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class PIDRollerIOSim implements RollerIO {

    private final @NotNull TunableTalonFX rollerMotor;
    private final @Nullable TunableTalonFX followerMotor;
    private final @NotNull Slot0Configs rollerPID;
    private final @NotNull LoggedNetworkNumber intakeSpeed;
    private final @NotNull LoggedNetworkNumber outtakeSpeed;

    public PIDRollerIOSim(AbstractDriveTrainSimulation driveSim) {
        this.rollerPID = new Slot0Configs();
        this.rollerPID.kP = IntakeConstants.RollerConstants.PIDF.kP;
        this.rollerPID.kI = IntakeConstants.RollerConstants.PIDF.kI;
        this.rollerPID.kD = IntakeConstants.RollerConstants.PIDF.kD;

        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
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

        this.rollerMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kRollerLeaderMotorID, "rio", "Intake/RollerPID", this.rollerPID);
        this.rollerMotor.applyConfiguration(config);

        TalonFXSimState rollerMotorSim = this.rollerMotor.getSimState();
        rollerMotorSim.setMotorType(MotorType.KrakenX60);

        this.intakeSpeed = new LoggedNetworkNumber(
                "Intake/Roller/IntakeSpeed", IntakeConstants.RollerConstants.kIntakeSpeed.in(RPM));
        this.outtakeSpeed = new LoggedNetworkNumber(
                "Intake/Roller/OuttakeSpeed", IntakeConstants.RollerConstants.kOuttakeSpeed.in(RPM));

        if (IntakeConstants.RollerConstants.kfollowerEnabled) {
            this.followerMotor = new TunableTalonFX(
                    Constants.CANIDs.MotorIDs.kRollerFollowerMotorID, "rio", "Intake/RollerFollower");
            this.followerMotor.getConfigurator().apply(config);
            this.followerMotor.getSimState().setMotorType(MotorType.KrakenX60);
        } else {
            this.followerMotor = null;
        }
    }

    public void setRollerSpeed(@NotNull AngularVelocity speed) {
        this.rollerMotor.setControl(new VelocityVoltage(speed));
        this.setFollower();
    }

    @Override
    public void stop() {
        this.rollerMotor.stopMotor();
        if (null != followerMotor) this.followerMotor.stopMotor();
    }

    @Override
    public void start() {
        this.setRollerSpeed(RPM.of(this.intakeSpeed.get()));
    }

    @Override
    public void outtake() {
        this.setRollerSpeed(RPM.of(this.outtakeSpeed.get()));
    }

    @Override
    public int getIntakedFuel() {
        // Real IO returns 0 for PID roller
        return 0;
    }

    @Override
    public void setMode(NeutralModeValue mode) {
        this.rollerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
    }

    @Override
    public void setMotorPercentage(double percent) {
        this.rollerMotor.set(percent);
    }

    @Override
    public void updateInputs(RollerIO.@NotNull RollerIOInputs inputs) {
        inputs.leaderSpeedPercentile = this.rollerMotor.get();
        inputs.leaderAppliedVolts = this.rollerMotor.getMotorVoltage().getValue();
        inputs.leaderVelocity = this.rollerMotor.getVelocity().getValue();
        inputs.leaderStatorCurrent = this.rollerMotor.getStatorCurrent().getValue();
        inputs.leaderMotorTemp = this.rollerMotor.getDeviceTemp().getValue();

        if (null != followerMotor) {
            inputs.followerSpeedPercentile = this.followerMotor.get();
            inputs.followerAppliedVolts = this.followerMotor.getMotorVoltage().getValue();
            inputs.followerVelocity = this.followerMotor.getVelocity().getValue();
            inputs.followerStatorCurrent = this.followerMotor.getStatorCurrent().getValue();
            inputs.followerMotorTemp = this.followerMotor.getDeviceTemp().getValue();
        }
    }

    @Override
    public void periodic() {
        this.rollerMotor.updateTunableGains();
        if (null != followerMotor) this.followerMotor.updateTunableGains();
    }

    public void setFollower() {
        if (null != followerMotor) {
            // Simulate follower by mirroring leader output, respecting configured alignment
            double leaderOut = this.rollerMotor.get();
            if (IntakeConstants.RollerConstants.MotorConfig.kFollowerInverted == MotorAlignmentValue.Opposed) {
                this.followerMotor.set(-leaderOut);
            } else {
                this.followerMotor.set(leaderOut);
            }
        }
    }
}
