package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;

public class IntakeIOSim implements IntakeIO {

    public final TalonFXSimState intakeMotorControllerSim;
    public final FlywheelSim flywheelSim;
    public final Mechanism2d flatSim;
    public final MechanismLigament2d roller;
    public final MechanismRoot2d flatSimRoot;

    public IntakeIOSim() {
        intakeMotorControllerSim = new TalonFX(IntakeConstants.MotorIDs.ROLLER_MOTOR_ID).getSimState();
        intakeMotorControllerSim.setMotorType(MotorType.KrakenX60);
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getKrakenX60(1),
                        RollerConstants.ROLLER_MOI.in(KilogramMetersSquaredPerSecond),
                        RollerConstants.ROLLER_GEARING),
                DCMotor.getKrakenX60(1));

        flatSim = new Mechanism2d(5, 5);
        flatSimRoot = flatSim.getRoot("Roller", 3, 3);
        roller = new MechanismLigament2d("Roller", 1, 0.0);
        roller.setLineWeight(30);
        flatSimRoot.append(roller);
    }

    @Override
    public void setSpeed(double speed) {
        intakeMotorControllerSim.setSupplyVoltage(speed * 12);
    }

    @Override
    public void stop() {
        intakeMotorControllerSim.setSupplyVoltage(0.0);
    }

    @Override
    public void periodic() {
        flywheelSim.setInputVoltage(
                intakeMotorControllerSim.getMotorVoltage() / 12.0 * RobotController.getBatteryVoltage());
        flywheelSim.update(0.02);
    }
}
