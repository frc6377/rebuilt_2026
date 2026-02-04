package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class RollerIOSim implements RollerIO {

    private final TalonFX rollerMotor;
    private final TalonFXConfiguration rollerMotorConfig;
    private final TalonFXSimState intakeMotorSim;
    private final IntakeSimulation intakeSim;

    public RollerIOSim(AbstractDriveTrainSimulation driveSim) {
        rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        rollerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        rollerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerMotorID);
        intakeMotorSim = rollerMotor.getSimState();
        intakeMotorSim.setMotorType(MotorType.KrakenX60);
        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel", driveSim, Inches.of(24), Inches.of(10), IntakeSimulation.IntakeSide.FRONT, 99);
    }

    @Override
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void start() {
        setRollerSpeed(RollerConstants.kIntakeSpeed);
        intakeSim.startIntake();
    }

    @Override
    public void stop() {
        setRollerSpeed(0);
        intakeSim.stopIntake();
    }

    @Override
    public void outtake() {
        setRollerSpeed(RollerConstants.kOuttakeSpeed);
        intakeSim.removeObtainedGamePieces(SimulatedArena.getInstance());
    }

    @Override
    public void updateInputs(RollerIO.IntakeIOInputs inputs) {
        inputs.rollerSpeedPercentile = intakeMotorSim.getMotorVoltage() / RobotController.getBatteryVoltage();
        inputs.rollerAppliedVolts = intakeMotorSim.getMotorVoltageMeasure();
    }
}
