package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunablePIDController;
import frc.robot.util.TunableTalonFX;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ExtenderIOReal implements ExtenderIO {

    private final TunableTalonFX extenderMotor;
    private final DutyCycleEncoder extenderEncoder;
    private final TunablePIDController extenderPid;
    private boolean pidEnabled = true;
    private final LoggedNetworkNumber extenderStowAngle;
    private final LoggedNetworkNumber extenderIntakeAngle;
    private final LoggedNetworkNumber extenderTolerance;
    private final LoggedNetworkNumber extenderSiftAngleOne;
    private final LoggedNetworkNumber extenderSiftAngleTwo;
    private final LoggedNetworkNumber extenderCustomAngleOne;
    private final LoggedNetworkNumber extenderCustomAngleTwo;

    public ExtenderIOReal() {

        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(ExtenderConstants.MotorConfig.kInverted)
                        .withNeutralMode(ExtenderConstants.MotorConfig.kNeutralMode))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(ExtenderConstants.MotorConfig.kRampPeriod))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(ExtenderConstants.MotorConfig.kStatorCurrentLimitExtender));

        extenderMotor = new TunableTalonFX(Constants.CANIDs.MotorIDs.kExtenderMotorID, "rio", "Extender");
        extenderMotor.getConfigurator().apply(config);

        extenderEncoder = new DutyCycleEncoder(
                Constants.CANIDs.SensorIDs.kExtenderEncoderCANID,
                1.0,
                ExtenderConstants.kExtenderZeroAngle.in(Rotations));
        extenderEncoder.setInverted(true);

        extenderPid = new TunablePIDController(
                "Intake/Extender/ExtenderPID",
                () -> getPosition().in(Degrees),
                percent -> extenderMotor.set(-percent),
                ExtenderConstants.kExtenderConstraints);

        extenderPid.addPreset("default", ExtenderConstants.PIDF.normalPID);
        extenderPid.addPreset("float", ExtenderConstants.PIDF.floatPID);

        extenderPid.getPIDController().enableContinuousInput(0, 359);

        extenderStowAngle =
                new LoggedNetworkNumber("Intake/Extender/StowAngle", ExtenderConstants.kExtenderStowAngle.in(Degrees));
        extenderIntakeAngle = new LoggedNetworkNumber(
                "Intake/Extender/IntakingAngle", ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        extenderTolerance =
                new LoggedNetworkNumber("Intake/Extender/Tolerance", ExtenderConstants.kExtenderTolerance.in(Degrees));
        extenderSiftAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/Sifting/SiftAngleOne", ExtenderConstants.kExtenderSiftAngleOne.in(Degrees));
        extenderSiftAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/Sifting/SiftAngleTwo", ExtenderConstants.kExtenderSiftAngleTwo.in(Degrees));
        extenderCustomAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/CustomAngleOne", ExtenderConstants.kExtenderCustomAngleOne.in(Degrees));
        extenderCustomAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/CustomAngleTwo", ExtenderConstants.kExtenderCustomAngleTwo.in(Degrees));

        extenderStowAngle.set(ExtenderConstants.kExtenderStowAngle.in(Degrees));
        extenderIntakeAngle.set(ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        extenderTolerance.set(ExtenderConstants.kExtenderTolerance.in(Degrees));
        extenderSiftAngleOne.set(ExtenderConstants.kExtenderSiftAngleOne.in(Degrees));
        extenderSiftAngleTwo.set(ExtenderConstants.kExtenderSiftAngleTwo.in(Degrees));
        extenderCustomAngleOne.set(ExtenderConstants.kExtenderCustomAngleOne.in(Degrees));
        extenderCustomAngleTwo.set(ExtenderConstants.kExtenderCustomAngleTwo.in(Degrees));
        extenderPid.applyPreset("default");

        setPidEnabled(false);
    }

    public void setPosition(Angle position) {
        setPidEnabled(true);
        setMode(NeutralModeValue.Brake);

        extenderPid.applyPreset("default");
        extenderPid.setSetpoint(position.in(Degrees));
    }

    public Angle getPosition() {
        return Degrees.of(Rotations.of(extenderEncoder.get()).in(Degrees));
    }

    public boolean isAtAngle(Angle angle) {
        return Math.abs((getPosition().minus(angle)).in(Degrees)) < extenderTolerance.get();
    }

    @Override
    public Current getCurrent() {
        return extenderMotor.getStatorCurrent().getValue();
    }

    @Override
    public void extend() {
        setPosition(Degrees.of(extenderIntakeAngle.get()));
    }

    @Override
    public void retract() {
        setPosition(Degrees.of(extenderStowAngle.get()));
    }

    @Override
    public BooleanSupplier isExtended() {
        return () -> isAtAngle(Degrees.of(extenderIntakeAngle.get()));
    }

    @Override
    public BooleanSupplier isRetracted() {
        return () -> isAtAngle(Degrees.of(extenderStowAngle.get()));
    }

    @Override
    public BooleanSupplier atTarget() {
        return () -> isAtAngle(Degrees.of(extenderPid.getSetpoint()));
    }

    @Override
    public void goToSiftAngleOne() {
        setPosition(Degrees.of(extenderSiftAngleOne.get()));
    }

    @Override
    public void goToSiftAngleTwo() {
        setPosition(Degrees.of(extenderSiftAngleTwo.get()));
    }

    @Override
    public void goToCustomAngleOne() {
        setPosition(Degrees.of(extenderCustomAngleOne.get()));
    }

    @Override
    public void goToCustomAngleTwo() {
        setPosition(Degrees.of(extenderCustomAngleTwo.get()));
    }

    @Override
    public void toggleSift() {
        if (Degrees.of(extenderPid.getSetpoint()).equals(Degrees.of(extenderSiftAngleOne.get()))) {
            goToSiftAngleTwo();
        } else {
            goToSiftAngleOne();
        }
    }

    @Override
    public void stop() {
        pidEnabled = false;
        extenderMotor.stopMotor();
    }

    @Override
    public void setPidEnabled(boolean enabled) {
        pidEnabled = enabled;
    }

    @Override
    public void setMode(NeutralModeValue mode) {
        extenderMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
    }

    @Override
    public void setMotorPercentage(double percent) {
        setPidEnabled(false);
        extenderMotor.set(percent);
    }

    @Override
    public void toggle() {
        double stowDeg = extenderStowAngle.get();
        if (Math.abs(Degrees.of(extenderPid.getSetpoint())
                        .minus(Degrees.of(stowDeg))
                        .in(Degrees))
                < extenderTolerance.get()) {
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
        inputs.setpoint = Degrees.of(extenderPid.getSetpoint());
        inputs.velocity = extenderMotor.getVelocity().getValue();
        inputs.motorVoltage = Volts.of(extenderMotor.getMotorVoltage().getValueAsDouble());
        inputs.motorCurrent = extenderMotor.getStatorCurrent().getValue();
        inputs.motorTemp = extenderMotor.getDeviceTemp().getValue();
        inputs.atTarget = atTarget().getAsBoolean();
        inputs.rawEncoderDegrees = Rotations.of(extenderEncoder.get()).in(Degrees);
    }

    @Override
    public void periodic() {
        extenderPid.updateTunableGains();
        if (pidEnabled) {
            extenderPid.runPid();
        }
        if (ExtenderConstants.floatEnabled) {
            if (getPosition().gte(ExtenderConstants.kExtenderFloatLimit)
                    && atTarget().getAsBoolean()) {
                extenderPid.applyPreset("float");
                setMode(NeutralModeValue.Coast);
            }
        }
    }
}
