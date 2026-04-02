package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import frc.robot.util.TunableTalonFX;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class PIDRollerIOSim implements RollerIO {

    private final TunableTalonFX rollerMotor;
    private final TalonFXSimState rollerMotorSim;
    private final IntakeSimulation intakeSim;
    private final Slot0Configs rollerPID;

    public PIDRollerIOSim(AbstractDriveTrainSimulation driveSim) {
        rollerPID = new Slot0Configs();
        rollerPID.kP = IntakeConstants.RollerConstants.PIDF.kP;
        rollerPID.kI = IntakeConstants.RollerConstants.PIDF.kI;
        rollerPID.kD = IntakeConstants.RollerConstants.PIDF.kD;

        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(RollerConstants.MotorConfig.kInvertedSim)
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

        rollerMotorSim = rollerMotor.getSimState();
        rollerMotorSim.setMotorType(MotorType.KrakenX60);
        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveSim,
                IntakeConstants.kIntakeWidth,
                IntakeConstants.kIntakeExtension,
                IntakeConstants.kIntakeSide,
                IntakeConstants.kIntakeCapacity);
    }

    public void setRollerSpeed(AngularVelocity speed) {
        rollerMotor.setControl(new VelocityVoltage(0).withVelocity(speed));
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
        intakeSim.stopIntake();
    }

    @Override
    public void start() {
        setRollerSpeed(IntakeConstants.RollerConstants.kIntakeSpeed);
        intakeSim.startIntake();
    }

    @Override
    public void outtake() {
        setRollerSpeed(IntakeConstants.RollerConstants.kOuttakeSpeed);
        intakeSim.removeObtainedGamePieces(SimulatedArena.getInstance());
    }

    @Override
    public int getIntakedFuel() {
        return intakeSim.getGamePiecesAmount();
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
        inputs.rollerSpeedPercentile = rollerMotorSim.getMotorVoltage() / RobotController.getBatteryVoltage();
        inputs.rollerAppliedVolts = rollerMotorSim.getMotorVoltageMeasure();
        inputs.rollerVelocity = rollerMotor.getVelocity().getValue();
        inputs.statorCurrent = rollerMotor.getStatorCurrent().getValue();
        inputs.motorTemp = Celsius.of(25.0);
    }

    @Override
    public void periodic() {
        rollerMotor.updateTunableGains();
    }
}
