package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANIDs;
import frc.robot.util.TunableTalonFX;

public class ClimberIOReal implements ClimberIO {
    final TunableTalonFX climbMotor1;
    final DutyCycleEncoder climbEncoder;
    final Slot0Configs climberPID;

    public ClimberIOReal() {
        climbMotor1 = new TunableTalonFX(CANIDs.kClimbMotor1ID, "rio", "ClimbMotor1");
        climbEncoder = new DutyCycleEncoder(CANIDs.kClimbEncoderID);
        tryUntilOk(5, () -> climbMotor1.getConfigurator().apply(ClimbConstants.kClimbMotorConfig, 0.25));
        climbMotor1.setPosition(climbEncoder.get());

        climberPID = new Slot0Configs();
        climberPID.kP = ClimbConstants.PIDF.kP;
        climberPID.kI = ClimbConstants.PIDF.kI;
        climberPID.kD = ClimbConstants.PIDF.kD;
    
    }

    @Override
    public void goToHeight(Distance height) {
        climbMotor1.setControl(new PositionVoltage(height.times(ClimbConstants.kClimbGearRatio)
                .div(ClimbConstants.kElevatorDrumCircumference)
                .times(Rotations.one())));
    }

    @Override
    public void stop() {
        climbMotor1.stopMotor();
    }

    @Override
    public void set(double percent) {
        climbMotor1.set(percent);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double positionRotations = climbMotor1.getPosition().getValue().in(Rotations);
        inputs.height = ClimbConstants.kElevatorDrumCircumference
                .times(positionRotations)
                .div(ClimbConstants.kClimbGearRatio);
    }

    @Override
    public void periodic() {
        // Update the encoder position to match the absolute encoder
        climbMotor1.updateTunableGains();
    }
}
