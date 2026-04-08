package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunablePIDController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ExtenderIOReal implements ExtenderIO {

    private static final String NT_EXTENDER = "Intake/Extender";

    private final TalonFX extenderMotor;
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

        extenderMotor = new TalonFX(Constants.CANIDs.MotorIDs.kExtenderMotorID);
        extenderMotor.getConfigurator().apply(config);

        extenderEncoder = new DutyCycleEncoder(
                Constants.CANIDs.SensorIDs.kExtenderEncoderCANID,
                1.0,
                ExtenderConstants.kExtenderZeroAngle.in(Rotations));
        extenderEncoder.setInverted(true);

        extenderPid = new TunablePIDController(
                NT_EXTENDER + "/ExtenderPID", () -> getPosition().in(Radians), percent -> extenderMotor.set(-percent));

        extenderPid.addPreset("default", ExtenderConstants.PIDF.normalPID);
        extenderPid.addPreset("float", ExtenderConstants.PIDF.floatPID);

        extenderPid.getPIDController().enableContinuousInput(0, 360);

        extenderStowAngle = tunableDegrees(NT_EXTENDER + "/StowAngle", ExtenderConstants.kExtenderStowAngle);
        extenderIntakeAngle = tunableDegrees(NT_EXTENDER + "/IntakingAngle", ExtenderConstants.kExtenderIntakeAngle);
        extenderTolerance = tunableDegrees(NT_EXTENDER + "/Tolerance", ExtenderConstants.kExtenderTolerance);
        extenderSiftAngleOne = tunableDegrees(NT_EXTENDER + "/Sifting/SiftAngleOne", ExtenderConstants.kExtenderSiftAngleOne);
        extenderSiftAngleTwo = tunableDegrees(NT_EXTENDER + "/Sifting/SiftAngleTwo", ExtenderConstants.kExtenderSiftAngleTwo);
        extenderCustomAngleOne = tunableDegrees(NT_EXTENDER + "/CustomAngleOne", ExtenderConstants.kExtenderCustomAngleOne);
        extenderCustomAngleTwo = tunableDegrees(NT_EXTENDER + "/CustomAngleTwo", ExtenderConstants.kExtenderCustomAngleTwo);
    }

    /** LoggedNetworkNumber for a degree tunable; initial value matches constants. */
    private static LoggedNetworkNumber tunableDegrees(String tablePath, Angle degreesValue) {
        return new LoggedNetworkNumber(tablePath, degreesValue.in(Degrees));
    }

    /** PID setpoint is stored in radians (see setPosition). */
    private Angle getSetpointAngle() {
        return Radians.of(extenderPid.getSetpoint());
    }

    private boolean setpointWithinTolerance(Angle target) {
        return Math.abs(getSetpointAngle().minus(target).in(Degrees)) < extenderTolerance.get();
    }

    public void setPosition(Angle position) {
        setPidEnabled(true);
        setMode(NeutralModeValue.Brake);
        extenderPid.applyPreset("default");
        extenderPid.setSetpoint(position.in(Radians));
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
        return () -> isAtAngle(getSetpointAngle());
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
        if (setpointWithinTolerance(Degrees.of(extenderSiftAngleOne.get()))) {
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
        if (setpointWithinTolerance(Degrees.of(extenderStowAngle.get()))) {
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
        inputs.setpoint = getSetpointAngle();
        inputs.velocity = extenderMotor.getVelocity().getValue();
        inputs.motorVoltage = Volts.of(extenderMotor.getMotorVoltage().getValueAsDouble());
        inputs.motorCurrent = extenderMotor.getStatorCurrent().getValue();
        inputs.motorTemp = extenderMotor.getDeviceTemp().getValue();
        inputs.atTarget = atTarget().getAsBoolean();
        inputs.rawEncoderDegrees = Rotations.of(extenderEncoder.get())
                .plus(ExtenderConstants.kExtenderZeroAngle)
                .in(Degrees);
    }

    @Override
    public void periodic() {
        extenderPid.updateTunableGains();
        if (pidEnabled) {
            extenderPid.runPid();
        }
        maybeApplyFloatMode();
    }

    /** When enabled, soften holding at high angle after the arm reaches setpoint. */
    private void maybeApplyFloatMode() {
        if (!ExtenderConstants.floatEnabled) {
            return;
        }
        if (getPosition().gte(ExtenderConstants.kExtenderFloatLimit) && atTarget().getAsBoolean()) {
            extenderPid.applyPreset("float");
            setMode(NeutralModeValue.Coast);
        }
    }
}
