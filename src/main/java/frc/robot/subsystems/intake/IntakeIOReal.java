package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {

    public final TalonFX intakeMotor;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(IntakeConstants.MotorIDs.ROLLER_MOTOR_ID);
    }

    @Override
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                "Intake/RollerMotorVoltage",
                intakeMotor.getMotorVoltage().getValue().in(Volts));
        Logger.recordOutput(
                "Intake/RollerMotorSpeedPercentile",
                intakeMotor.getSupplyVoltage().getValue().in(Volts) / 12.0);
    }
}
