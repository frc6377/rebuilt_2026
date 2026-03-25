package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RollerIOSim implements RollerIO {

    private final TalonFX rollerMotor;
    private final TalonFXConfiguration rollerMotorConfig;
    private final TalonFXSimState intakeMotorSim;
    private final IntakeSimulation intakeSim;
    private final CurrentLimitsConfigs currentConfig;
    // dashboard-tunable roller speeds
    private final LoggedNetworkNumber kRollerIntakePercent;
    private final LoggedNetworkNumber kRollerOuttakePercent;

    public RollerIOSim(AbstractDriveTrainSimulation driveSim) {
        currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = RollerConstants.MotorConfig.kStatorCurrentLimit.in(Amps);

        rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = RollerConstants.MotorConfig.kRampPeriod;
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerMotorID);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
        rollerMotor.getConfigurator().apply(currentConfig);
        intakeMotorSim = rollerMotor.getSimState();
        intakeMotorSim.setMotorType(MotorType.KrakenX60);
        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveSim,
                IntakeConstants.kIntakeWidth,
                IntakeConstants.kIntakeExtension,
                IntakeConstants.kIntakeSide,
                IntakeConstants.kIntakeCapacity);

        kRollerIntakePercent = new LoggedNetworkNumber("Intake/Roller/IntakePercent", RollerConstants.kIntakePercent);
        kRollerOuttakePercent =
                new LoggedNetworkNumber("Intake/Roller/OuttakePercent", RollerConstants.kOuttakePercent);
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void start() {
        setRollerSpeed(kRollerIntakePercent.get());
        intakeSim.startIntake();
    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
        intakeSim.stopIntake();
    }

    @Override
    public void outtake() {
        setRollerSpeed(kRollerOuttakePercent.get());
        intakeSim.removeObtainedGamePieces(SimulatedArena.getInstance());
    }

    @Override
    public int getIntakedFuel() {
        return intakeSim.getGamePiecesAmount();
    }

    @Override
    public void updateInputs(RollerIO.RollerIOInputs inputs) {
        inputs.rollerSpeedPercentile = intakeMotorSim.getMotorVoltage() / RobotController.getBatteryVoltage();
        inputs.rollerAppliedVolts = intakeMotorSim.getMotorVoltageMeasure();
        inputs.rollerVelocity = rollerMotor.getVelocity().getValue();
        inputs.motorTemp = Celsius.of(25.0);
    }
}
