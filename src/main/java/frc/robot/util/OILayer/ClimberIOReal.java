package frc.robot.util.OILayer;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {
    final TalonFX climbMotor1;
    private final TalonFX climbMotor2;

    public ClimberIOReal() {
        climbMotor1 = new TalonFX(Constants.CANIDs.CLIMB_MOTOR_1_ID);
        climbMotor2 = new TalonFX(Constants.CANIDs.CLIMB_MOTOR_2_ID);

        tryUntilOk(5, () -> climbMotor1.getConfigurator().apply(ClimbConstants.kClimbMotorConfig, 0.25));
        tryUntilOk(5, () -> climbMotor2.getConfigurator().apply(ClimbConstants.kClimbMotorConfig, 0.25));
    }

    @Override
    public void goToHeight(Angle height) {
        climbMotor1.setControl(new PositionVoltage(height));
    }

    public void stop() {
        climbMotor1.stopMotor();
        climbMotor2.stopMotor();
    }

    @Override
    public void set(double percent) {
        climbMotor1.set(percent);
        climbMotor2.set(percent);
    }
}
