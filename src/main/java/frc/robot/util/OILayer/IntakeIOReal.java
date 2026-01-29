package frc.robot.util.OILayer;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    public final TalonFX intakeMotor;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(Constants.MotorIDs.ROLLER_MOTOR_ID);
    }

    public void setRollerSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public void start() {
        setRollerSpeed(IntakeConstants.RollerConstants.INTAKE_SPEED);
    }

    public void outtake() {
        setRollerSpeed(IntakeConstants.RollerConstants.OUTAKE_SPEED);
    }

    public void updateInputs(IntakeIOReal.IntakeIOInputs inputs) {
        inputs.rollerSpeedPercentile = intakeMotor.get();
        inputs.rollerAppliedVolts = intakeMotor.getMotorVoltage().getValue();
    }
}