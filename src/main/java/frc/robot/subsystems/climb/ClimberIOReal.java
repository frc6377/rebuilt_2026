package frc.robot.subsystems.climb;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CANIDs;

public class ClimberIOReal implements ClimberIO {
    final TalonFX climbMotor1;
    private final TalonFX climbMotor2;

    public ClimberIOReal() {
        climbMotor1 = new TalonFX(CANIDs.kClimbMotor1ID);
        climbMotor2 = new TalonFX(CANIDs.kClimbMotor2ID);

        tryUntilOk(5, () -> climbMotor1.getConfigurator().apply(ClimbConstants.kClimbMotorConfig, 0.25));
        tryUntilOk(5, () -> climbMotor2.getConfigurator().apply(ClimbConstants.kClimbMotorConfig, 0.25));
    }

    @Override
    public void goToHeight(Angle height) {
        climbMotor1.setControl(new PositionVoltage(height));
    }

    @Override
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
