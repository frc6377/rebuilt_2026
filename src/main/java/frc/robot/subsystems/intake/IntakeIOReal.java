package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

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
}
