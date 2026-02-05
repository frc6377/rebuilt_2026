package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.CANIDs;

public class ClimberIOReal implements ClimberIO {
    final TalonFX climbMotor1;

    public ClimberIOReal() {
        climbMotor1 = new TalonFX(CANIDs.kClimbMotor1ID);

        tryUntilOk(5, () -> climbMotor1.getConfigurator().apply(ClimbConstants.kClimbMotorConfig, 0.25));
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
}
