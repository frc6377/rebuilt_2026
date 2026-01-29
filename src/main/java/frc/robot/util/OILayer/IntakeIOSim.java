package frc.robot.util.OILayer;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {

    public final TalonFX intakeMotor;
    public final TalonFXSimState intakeMotorSim;
    public final IntakeSimulation intakeSim;

    public IntakeIOSim(AbstractDriveTrainSimulation driveSim) {
        intakeMotor = new TalonFX(Constants.MotorIDs.ROLLER_MOTOR_ID);
        intakeMotorSim = intakeMotor.getSimState();
        intakeMotorSim.setMotorType(MotorType.KrakenX60);
        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel", driveSim, Inches.of(24), Inches.of(10), IntakeSimulation.IntakeSide.FRONT, 99);
    }

    @Override
    public void setRollerSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void start() {
        setRollerSpeed(IntakeConstants.RollerConstants.INTAKE_SPEED);
        intakeSim.startIntake();
    }

    public void stop() {
        setRollerSpeed(0);
        intakeSim.stopIntake();
    }

    public void outtake() {
        setRollerSpeed(IntakeConstants.RollerConstants.OUTAKE_SPEED);
        intakeSim.removeObtainedGamePieces(SimulatedArena.getInstance());
    }

    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        inputs.rollerSpeedPercentile = intakeMotorSim.getMotorVoltage() / RobotController.getBatteryVoltage();
        inputs.rollerAppliedVolts = intakeMotorSim.getMotorVoltageMeasure();
    }
}