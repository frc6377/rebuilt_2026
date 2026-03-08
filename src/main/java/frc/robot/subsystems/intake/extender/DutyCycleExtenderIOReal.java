package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunablePIDController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class DutyCycleExtenderIOReal implements ExtenderIO {

    private final TalonFX extenderMotor;
    private final DutyCycleEncoder extenderEncoder;
    private final CurrentLimitsConfigs currentConfig;
    private final TalonFXConfiguration extenderMotorConfig;
    private final TunablePIDController extenderPid;
    private Angle setpoint;
    // Logged network numbers for tuning/monitoring extender angles (no "NN" suffix
    // per request)
    private final LoggedNetworkNumber kExtenderStowAngle;
    private final LoggedNetworkNumber kExtenderIntakeAngle;
    private final LoggedNetworkNumber kExtenderMaxAngle;
    // private final LoggedNetworkNumber kExtenderMinAngle;
    private final LoggedNetworkNumber kExtenderTolerance;
    private final LoggedNetworkNumber kExtenderSiftAngleOne;
    private final LoggedNetworkNumber kExtenderSiftAngleTwo;
    private final LoggedNetworkNumber kExtenderDownSpeed;
    private final LoggedNetworkNumber kExtenderZeroCurrentLimit;

    public DutyCycleExtenderIOReal() {
        this.setpoint = Degrees.of(0.0);

        extenderMotor = new TalonFX(Constants.CANIDs.MotorIDs.kExtenderMotorID);

        currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = ExtenderConstants.MotorConfig.kStatorCurrentLimitExtender.in(Amps);

        extenderMotorConfig = new TalonFXConfiguration();
        extenderMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ExtenderConstants.MotorConfig.kRampPeriod;
        extenderMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extenderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        extenderMotor.getConfigurator().apply(extenderMotorConfig);
        extenderMotor.getConfigurator().apply(currentConfig);

        extenderEncoder = new DutyCycleEncoder(
                Constants.CANIDs.SensorIDs.kExtenderEncoderCANID,
                1.0,
                ExtenderConstants.kExtenderZeroAngle.in(Rotations));

        extenderPid = new TunablePIDController(
                "Intake/ExtenderPID",
                ExtenderConstants.PIDF.kP,
                ExtenderConstants.PIDF.kI,
                ExtenderConstants.PIDF.kD,
                () -> getPosition().in(Rotations),
                percent -> extenderMotor.set(percent));

        kExtenderStowAngle =
                new LoggedNetworkNumber("Intake/Extender/StowAngle", ExtenderConstants.kExtenderStowAngle.in(Degrees));
        kExtenderIntakeAngle = new LoggedNetworkNumber(
                "Intake/Extender/IntakeAngle", ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        kExtenderMaxAngle =
                new LoggedNetworkNumber("Intake/Extender/MaxAngle", ExtenderConstants.kExtenderMaxAngle.in(Degrees));
        // kExtenderMinAngle =
        // new LoggedNetworkNumber("Intake/Extender/MinAngle",
        // ExtenderConstants.kExtenderMinAngle.in(Degrees));
        kExtenderTolerance =
                new LoggedNetworkNumber("Intake/Extender/Tolerance", ExtenderConstants.kExtenderTolerance.in(Degrees));
        kExtenderSiftAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleOne", ExtenderConstants.kExtenderSiftAngleOne.in(Degrees));
        kExtenderSiftAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleTwo", ExtenderConstants.kExtenderSiftAngleTwo.in(Degrees));
        kExtenderDownSpeed = new LoggedNetworkNumber("Intake/Extender/DownSpeed", ExtenderConstants.kDownSpeed);
        kExtenderZeroCurrentLimit =
                new LoggedNetworkNumber("Intake/Extender/DownSpeed", ExtenderConstants.zeroCurrentLimit.in(Amps));
    }

    public void setPosition(Angle position) {
        this.setpoint = position;
        Logger.recordOutput("Intake/Extender/SetpointDegrees", position);
        extenderPid.setSetpoint(position.in(Rotations));
    }

    public Angle getPosition() {
        return Rotations.of(extenderEncoder.get());
    }

    public boolean isAtAngle(Angle angle) {
        return Math.abs((getPosition().minus(angle)).in(Degrees)) < kExtenderTolerance.get();
    }

    @Override
    public void currentRunShoot(double volts) {
        extenderMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void zero() {
        extenderMotor.setPosition(0.0);
    }

    @Override
    public AngularVelocity getVelocity() {
        return extenderMotor.getVelocity().getValue();
    }

    @Override
    public Current getCurrent() {
        return extenderMotor.getStatorCurrent().getValue();
    }

    @Override
    public void extend() {
        setPosition(Degrees.of(kExtenderIntakeAngle.get()));
    }

    @Override
    public void retract() {
        setPosition(Degrees.of(kExtenderStowAngle.get()));
    }

    @Override
    public BooleanSupplier isExtended() {
        return () -> isAtAngle(Degrees.of(kExtenderIntakeAngle.get()));
    }

    @Override
    public BooleanSupplier isRetracted() {
        return () -> isAtAngle(Degrees.of(kExtenderStowAngle.get()));
    }

    @Override
    public BooleanSupplier atTarget() {
        return () -> isAtAngle(setpoint);
    }

    @Override
    public void goToSiftAngleOne() {
        setPosition(Degrees.of(kExtenderSiftAngleOne.get()));
    }

    @Override
    public void goToSiftAngleTwo() {
        setPosition(Degrees.of(kExtenderSiftAngleTwo.get()));
    }

    @Override
    public void stop() {
        extenderMotor.stopMotor();
    }

    @Override
    public void setEncoderPosition(Angle position) {
        extenderMotor.setPosition(position);
    }

    @Override
    public void autoZero() {
        Slot0Configs zeroPID = new Slot0Configs();
        zeroPID.kP = 0.0;
        extenderMotor.getConfigurator().apply(zeroPID);
        extenderMotor.set(kExtenderDownSpeed.get());

        double currentAmps = extenderMotor.getStatorCurrent().getValue().in(Amps);
        double thresholdAmps = ExtenderConstants.zeroCurrentLimit.in(Amps);

        if (Math.abs(currentAmps) >= thresholdAmps) {
            extenderMotor.stopMotor();
            setEncoderPosition(Degrees.of(kExtenderMaxAngle.get()));
        }

        extenderMotor.getConfigurator().apply(this.extenderMotorConfig);
        goToSiftAngleOne();
    }

    @Override
    public void toggle() {
        if (Math.abs(setpoint.in(Degrees) - kExtenderStowAngle.get()) < 5.0) {
            extend();
        } else {
            retract();
        }
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
        inputs.isExtended = isExtended().getAsBoolean();
        inputs.isRetracted = isRetracted().getAsBoolean();
        inputs.position = getPosition();
        inputs.setpoint = setpoint;
        inputs.velocity = extenderMotor.getVelocity().getValue();
        inputs.motorVoltage = Volts.of(extenderMotor.getMotorVoltage().getValueAsDouble());
        inputs.motorCurrent = extenderMotor.getStatorCurrent().getValue();
        inputs.motorTemp = extenderMotor.getDeviceTemp().getValue();
        inputs.atTarget = atTarget().getAsBoolean();
    }

    @Override
    public void periodic() {
        extenderPid.updateTunableGains();
        extenderPid.runPid();
    }
}
