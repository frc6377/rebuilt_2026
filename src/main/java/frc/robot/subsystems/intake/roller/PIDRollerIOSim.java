package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
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
    private final TalonFXConfiguration rollerMotorConfig;
    private final TalonFXSimState rollerMotorSim;
    private final IntakeSimulation intakeSim;
    private final Slot0Configs rollerPID;
    private final CurrentLimitsConfigs currentConfig;

    public PIDRollerIOSim(AbstractDriveTrainSimulation driveSim) {

        currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = RollerConstants.MotorConfig.kStatorCurrentLimit.in(Amps);

        rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = RollerConstants.MotorConfig.kRampPeriod;
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerPID = new Slot0Configs();
        rollerPID.kP = IntakeConstants.RollerConstants.PIDF.kP;
        rollerPID.kI = IntakeConstants.RollerConstants.PIDF.kI;
        rollerPID.kD = IntakeConstants.RollerConstants.PIDF.kD;

        rollerMotor =
                new TunableTalonFX(Constants.CANIDs.MotorIDs.kRollerMotorID, "rio", "Intake/RollerPID", rollerPID);
        rollerMotor.applyConfiguration(rollerMotorConfig);
        rollerMotor.getConfigurator().apply(currentConfig);

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
