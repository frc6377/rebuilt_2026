package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunableTalonFX;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ExtenderIOReal implements ExtenderIO {

    private final TunableTalonFX extenderMotor;
    private final CurrentLimitsConfigs currentConfig;
    private final TalonFXConfiguration extenderMotorConfig;
    private final Slot0Configs extenderPID;
    private Angle setpoint;
    // Logged network numbers for tuning/monitoring extender angles (no "NN" suffix per request)
    private final LoggedNetworkNumber kExtenderStowAngle;
    private final LoggedNetworkNumber kExtenderIntakeAngle;
    // private final LoggedNetworkNumber kExtenderMaxAngle;
    // private final LoggedNetworkNumber kExtenderMinAngle;
    private final LoggedNetworkNumber kExtenderTolerance;
    private final LoggedNetworkNumber kExtenderSiftAngleOne;
    private final LoggedNetworkNumber kExtenderSiftAngleTwo;
    private final LoggedNetworkNumber kExtenderDownSpeed;

    public ExtenderIOReal() {
        this.setpoint = Degrees.of(0.0);

        extenderPID = new Slot0Configs();
        extenderPID.kP = ExtenderConstants.PIDF.kP;
        extenderPID.kI = ExtenderConstants.PIDF.kI;
        extenderPID.kD = ExtenderConstants.PIDF.kD;

        extenderMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kExtenderMotorID, "rio", "Intake/ExtenderPID", extenderPID);

        currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = ExtenderConstants.MotorConfig.kStatorCurrentLimitExtender.in(Amps);

        extenderMotorConfig = new TalonFXConfiguration();
        extenderMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ExtenderConstants.MotorConfig.kRampPeriod;
        extenderMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        extenderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        extenderMotor.applyConfiguration(extenderMotorConfig);
        extenderMotor.getConfigurator().apply(currentConfig);

        kExtenderStowAngle =
                new LoggedNetworkNumber("Intake/Extender/StowAngle", ExtenderConstants.kExtenderStowAngle.in(Degrees));
        kExtenderIntakeAngle = new LoggedNetworkNumber(
                "Intake/Extender/IntakeAngle", ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        // kExtenderMaxAngle =
        //         new LoggedNetworkNumber("Intake/Extender/MaxAngle", ExtenderConstants.kExtenderMaxAngle.in(Degrees));
        // kExtenderMinAngle =
        //         new LoggedNetworkNumber("Intake/Extender/MinAngle", ExtenderConstants.kExtenderMinAngle.in(Degrees));
        kExtenderTolerance =
                new LoggedNetworkNumber("Intake/Extender/Tolerance", ExtenderConstants.kExtenderTolerance.in(Degrees));
        kExtenderSiftAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleOne", ExtenderConstants.kExtenderSiftAngleOne.in(Degrees));
        kExtenderSiftAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleTwo", ExtenderConstants.kExtenderSiftAngleTwo.in(Degrees));
        kExtenderDownSpeed = new LoggedNetworkNumber("Intake/Extender/DownSpeed", ExtenderConstants.kDownSpeed);
    }

    public void setPosition(Angle position) {
        Logger.recordOutput("Intake/Extender/PositionDegrees", position);
        extenderMotor.setControl(new PositionVoltage(position.times(ExtenderConstants.kGearing)));
    }

    public Angle getPosition() {
        return Degrees.of(extenderMotor
                .getPosition()
                .getValue()
                .div(ExtenderConstants.kGearing)
                .in(Degrees));
    }

    public boolean isAtAngle(Angle angle) {
        return Math.abs((getPosition().minus(angle)).in(Degrees))
                < kExtenderTolerance.get() * ExtenderConstants.kGearing;
    }

    @Override
    public void zero() {
        extenderMotor.setPosition(0.0);
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
    public void goDown() {
        extenderMotor.set(kExtenderDownSpeed.get());
    }

    @Override
    public void toggle() {
        if (isRetracted().getAsBoolean()) {
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
        inputs.setpoint = Degrees.of(setpoint.in(Degrees));
        inputs.velocity = extenderMotor.getVelocity().getValue();
        inputs.motorVoltage = Volts.of(extenderMotor.getMotorVoltage().getValueAsDouble());
        inputs.motorCurrent = extenderMotor.getStatorCurrent().getValue();
        inputs.motorTemp = extenderMotor.getDeviceTemp().getValue();
        inputs.atTarget = atTarget().getAsBoolean();
    }

    @Override
    public void periodic() {
        extenderMotor.updateTunableGains();
    }
}
