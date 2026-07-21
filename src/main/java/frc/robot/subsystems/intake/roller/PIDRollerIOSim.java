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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class PIDRollerIOSim implements RollerIO {

    private final TunableTalonFX rollerMotor;
    private final TunableTalonFX followerMotor;
    private final TalonFXSimState rollerMotorSim;
    private final TalonFXSimState followerMotorSim;
    private final Slot0Configs rollerPID;
    private final LoggedNetworkNumber intakeSpeed;
    private final LoggedNetworkNumber outtakeSpeed;

    public PIDRollerIOSim(AbstractDriveTrainSimulation driveSim) {
        rollerPID = new Slot0Configs();
        rollerPID.kP = IntakeConstants.RollerConstants.PIDF.kP;
        rollerPID.kI = IntakeConstants.RollerConstants.PIDF.kI;
        rollerPID.kD = IntakeConstants.RollerConstants.PIDF.kD;

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

        rollerMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kRollerLeaderMotorID, "rio", "Intake/RollerPID", rollerPID);
        rollerMotor.applyConfiguration(config);

        rollerMotorSim = rollerMotor.getSimState();
        rollerMotorSim.setMotorType(MotorType.KrakenX60);

        intakeSpeed = new LoggedNetworkNumber(
                "Intake/Roller/IntakeSpeed", IntakeConstants.RollerConstants.kIntakeSpeed.in(RPM));
        outtakeSpeed = new LoggedNetworkNumber(
                "Intake/Roller/OuttakeSpeed", IntakeConstants.RollerConstants.kOuttakeSpeed.in(RPM));

        if (IntakeConstants.RollerConstants.kfollowerEnabled) {
            followerMotor = new TunableTalonFX(
                    Constants.CANIDs.MotorIDs.kRollerFollowerMotorID, "rio", "Intake/RollerFollower");
            followerMotor.getConfigurator().apply(config);
            followerMotorSim = followerMotor.getSimState();
            followerMotorSim.setMotorType(MotorType.KrakenX60);
        } else {
            followerMotor = null;
            followerMotorSim = null;
        }
    }

    public void setRollerSpeed(AngularVelocity speed) {
        rollerMotor.setControl(new VelocityVoltage(speed));
        setFollower();
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
        if (followerMotor != null) followerMotor.stopMotor();
    }

    @Override
    public void start() {
        setRollerSpeed(RPM.of(intakeSpeed.get()));
    }

    @Override
    public void outtake() {
        setRollerSpeed(RPM.of(outtakeSpeed.get()));
    }

    @Override
    public int getIntakedFuel() {
        // Real IO returns 0 for PID roller
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
        inputs.leaderSpeedPercentile = rollerMotor.get();
        inputs.leaderAppliedVolts = rollerMotor.getMotorVoltage().getValue();
        inputs.leaderVelocity = rollerMotor.getVelocity().getValue();
        inputs.leaderStatorCurrent = rollerMotor.getStatorCurrent().getValue();
        inputs.leaderMotorTemp = rollerMotor.getDeviceTemp().getValue();

        if (followerMotor != null) {
            inputs.followerSpeedPercentile = followerMotor.get();
            inputs.followerAppliedVolts = followerMotor.getMotorVoltage().getValue();
            inputs.followerVelocity = followerMotor.getVelocity().getValue();
            inputs.followerStatorCurrent = followerMotor.getStatorCurrent().getValue();
            inputs.followerMotorTemp = followerMotor.getDeviceTemp().getValue();
        }
    }

    @Override
    public void periodic() {
        rollerMotor.updateTunableGains();
        if (followerMotor != null) followerMotor.updateTunableGains();
    }

    public void setFollower() {
        if (followerMotor != null) {
            // Simulate follower by mirroring leader output, respecting configured alignment
            double leaderOut = rollerMotor.get();
            if (IntakeConstants.RollerConstants.MotorConfig.kFollowerInverted == MotorAlignmentValue.Opposed) {
                followerMotor.set(-leaderOut);
            } else {
                followerMotor.set(leaderOut);
            }
        }
    }
}
