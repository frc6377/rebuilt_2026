package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {

    public final TalonFX intakeMotor;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerMotorID);
    }

    @Override
    public void setRollerSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }

    @Override
    public void start() {
        setRollerSpeed(IntakeConstants.RollerConstants.kIntakeSpeed);
    }

    @Override
    public void outtake() {
        setRollerSpeed(IntakeConstants.RollerConstants.kOuttakeSpeed);
    }

    @Override
    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        inputs.rollerSpeedPercentile = intakeMotor.get();
        inputs.rollerAppliedVolts = Volts.of(intakeMotor.getMotorVoltage().getValueAsDouble());
    }
}
