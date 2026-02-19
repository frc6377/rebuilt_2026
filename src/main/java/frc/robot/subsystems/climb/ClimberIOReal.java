package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANIDs;
import frc.robot.util.TunableTalonFX;

public class ClimberIOReal implements ClimberIO {
    final TunableTalonFX climbMotor1;
    final DutyCycleEncoder climbEncoder;
    final Slot0Configs climberPID;
    final DigitalInput climberLimitSwitch;

    public ClimberIOReal() {
        climbMotor1 = new TunableTalonFX(CANIDs.kClimbMotor1ID, "rio", "ClimbMotor1");
        climbEncoder = new DutyCycleEncoder(CANIDs.kClimbEncoderID);
        climberLimitSwitch = new DigitalInput(CANIDs.kClimbLimitSwitchID);
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
        inputs.motorPosition = Rotations.of(positionRotations);
        inputs.height = ClimbConstants.kElevatorDrumCircumference
                .times(positionRotations)
                .div(ClimbConstants.kClimbGearRatio);

        inputs.appliedVoltage = Volts.of(climbMotor1.getMotorVoltage().getValueAsDouble());
        inputs.statorCurrent = Amps.of(climbMotor1.getStatorCurrent().getValueAsDouble());
        inputs.supplyCurrent = Amps.of(climbMotor1.getSupplyCurrent().getValueAsDouble());
        inputs.temperatureCelsius = climbMotor1.getDeviceTemp().getValueAsDouble();

        inputs.absoluteEncoderPosition = climbEncoder.get();

        inputs.motorConnected = climbMotor1.isAlive();
    }

    @Override
    public void periodic() {
        climbMotor1.updateTunableGains();
    }

    @Override
    public void resetToAbsolute() {
        climbMotor1.setPosition(climbEncoder.get());
    }
}
