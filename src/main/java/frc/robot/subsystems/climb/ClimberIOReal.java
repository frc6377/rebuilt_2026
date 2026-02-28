package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANIDs.MotorIDs;
import frc.robot.Constants.CANIDs.SensorIDs;
import frc.robot.util.TunableTalonFX;

public class ClimberIOReal implements ClimberIO {
    final TunableTalonFX climbMotor1;
    final DutyCycleEncoder climbEncoder;
    final Slot0Configs climberPID;
    final DigitalInput limitSwitch;

    public ClimberIOReal() {
        climbMotor1 = new TunableTalonFX(MotorIDs.kClimbMotor1ID, "rio", "ClimbMotor1");
        climbEncoder = new DutyCycleEncoder(SensorIDs.kClimbEncoderID);
        limitSwitch = new DigitalInput(ClimbConstants.kLimitSwitchPort);

        tryUntilOk(5, () -> climbMotor1.getConfigurator().apply(ClimbConstants.kClimbMotorConfigReal, 0.25));
        tryUntilOk(5, () -> climbMotor1.getConfigurator().apply(ClimbConstants.kLimitSwitchConfig, 0.25));

        climbMotor1.setPosition(climbEncoder.get());

        climberPID = new Slot0Configs();
        climberPID.kP = ClimbConstants.PIDF.kP;
        climberPID.kI = ClimbConstants.PIDF.kI;
        climberPID.kD = ClimbConstants.PIDF.kD;

        tryUntilOk(5, () -> climbMotor1.getConfigurator().apply(climberPID, 0.25));
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

        inputs.limitSwitchPressed = !limitSwitch.get();
    }

    @Override
    public Distance getHeight() {
        return ClimbConstants.kElevatorDrumCircumference
                .times(climbMotor1.getPosition().getValue().in(Rotations))
                .div(ClimbConstants.kClimbGearRatio);
    }

    @Override
    public void disableSoftLimits() {
        SoftwareLimitSwitchConfigs config = new SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitEnable = false;
        config.ReverseSoftLimitEnable = false;
        tryUntilOk(5, () -> climbMotor1.getConfigurator().apply(config, 0.25));
    }

    @Override
    public void enableSoftLimits() {
        tryUntilOk(5, () -> climbMotor1.getConfigurator().apply(ClimbConstants.kLimitSwitchConfig, 0.25));
    }

    @Override
    public boolean limitHit() {
        return !limitSwitch.get();
    }

    @Override
    public void periodic() {
        climbMotor1.updateTunableGains();
    }

    @Override
    public void resetToAbsolute() {
        climbMotor1.setPosition(climbEncoder.get());
    }

    @Override
    public void zeroEncoder() {
        climbMotor1.setPosition(0);
    }
}
