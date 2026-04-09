package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunableTalonFX;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ExtenderIOReal implements ExtenderIO {

    private final TunableTalonFX extenderMotor;
    private final DutyCycleEncoder extenderEncoder;
    private boolean pidEnabled = true;
    private final LoggedNetworkNumber extenderStowAngle;
    private final LoggedNetworkNumber extenderIntakeAngle;
    private final LoggedNetworkNumber extenderTolerance;
    private final LoggedNetworkNumber extenderSiftAngleOne;
    private final LoggedNetworkNumber extenderSiftAngleTwo;
    private final LoggedNetworkNumber extenderCustomAngleOne;
    private final LoggedNetworkNumber extenderCustomAngleTwo;

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
    private final MotionMagicVoltage mmFloatRequest = new MotionMagicVoltage(0).withSlot(1);
    private double currentSetpointDegrees = ExtenderConstants.kExtenderStowAngle.in(Degrees);
    private boolean useFloatSlot = false;

    public ExtenderIOReal() {

        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(ExtenderConstants.MotorConfig.kInverted)
                        .withNeutralMode(ExtenderConstants.MotorConfig.kNeutralMode))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(ExtenderConstants.MotorConfig.kRampPeriod))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(ExtenderConstants.MotorConfig.kStatorCurrentLimitExtender))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(ExtenderConstants.MotionMagic.kCruiseVelocity)
                        .withMotionMagicAcceleration(ExtenderConstants.MotionMagic.kAcceleration)
                        .withMotionMagicJerk(ExtenderConstants.MotionMagic.kJerk))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0));

        config.Slot1 = new Slot1Configs()
                .withKP(ExtenderConstants.PIDF.floatPID.kP())
                .withKI(ExtenderConstants.PIDF.floatPID.kI())
                .withKD(ExtenderConstants.PIDF.floatPID.kD());

        extenderMotor = new TunableTalonFX(Constants.CANIDs.MotorIDs.kExtenderMotorID, "rio", "Intake/Extender/Motor");

        extenderEncoder = new DutyCycleEncoder(
                Constants.CANIDs.SensorIDs.kExtenderEncoderCANID,
                1.0,
                ExtenderConstants.kExtenderZeroAngle.in(Rotations));
        extenderEncoder.setInverted(true);

        extenderMotor
                .getTunableSlot0Configs()
                .withKP(ExtenderConstants.PIDF.normalPID.kP())
                .withKI(ExtenderConstants.PIDF.normalPID.kI())
                .withKD(ExtenderConstants.PIDF.normalPID.kD());

        extenderMotor.applyConfiguration(config);

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

        // initialize external angle
        extenderMotor.setPosition(Rotations.of(extenderEncoder.get()));
    }

    public void setPosition(Angle position) {
        setPidEnabled(true);
        setMode(NeutralModeValue.Brake);
        useFloatSlot = false;
        currentSetpointDegrees = position.in(Degrees);
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
        return () -> isAtAngle(Degrees.of(currentSetpointDegrees));
    }

    public void goToSiftAngleOne() {
        setPosition(Degrees.of(extenderSiftAngleOne.get()));
    }

    public void goToSiftAngleTwo() {
        setPosition(Degrees.of(extenderSiftAngleTwo.get()));
    }

    public void goToCustomAngleOne() {
        setPosition(Degrees.of(extenderCustomAngleOne.get()));
    }

    public void goToCustomAngleTwo() {
        setPosition(Degrees.of(extenderCustomAngleTwo.get()));
    }

    public void toggleSift() {
        if (Degrees.of(currentSetpointDegrees).equals(Degrees.of(extenderSiftAngleOne.get()))) {
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
        if (Math.abs(Degrees.of(currentSetpointDegrees)
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
        inputs.setpoint = Degrees.of(currentSetpointDegrees);
        inputs.velocity = extenderMotor.getVelocity().getValue();
        inputs.motorVoltage = Volts.of(extenderMotor.getMotorVoltage().getValueAsDouble());
        inputs.motorCurrent = extenderMotor.getStatorCurrent().getValue();
        inputs.motorTemp = extenderMotor.getDeviceTemp().getValue();
        inputs.atTarget = atTarget().getAsBoolean();
        inputs.rawEncoderDegrees = Rotations.of(extenderEncoder.get()).in(Degrees);
    }

    @Override
    public void periodic() {
        extenderMotor.updateTunableGains();
        // Update position every tick
        extenderMotor.setPosition(Rotations.of(extenderEncoder.get()).in(Rotations));

        if (pidEnabled) {
            double setpointRots = currentSetpointDegrees / 360.0;
            if (useFloatSlot) {
                extenderMotor.setControl(mmFloatRequest.withPosition(setpointRots));
            } else {
                extenderMotor.setControl(mmRequest.withPosition(setpointRots));
            }
        }

        if (ExtenderConstants.floatEnabled) {
            if (getPosition().gte(ExtenderConstants.kExtenderFloatLimit)
                    && atTarget().getAsBoolean()) {
                useFloatSlot = true;
                setMode(NeutralModeValue.Coast);
            } else if (getPosition().lt(ExtenderConstants.kExtenderFloatLimit)) {
                useFloatSlot = false;
                setMode(NeutralModeValue.Brake);
            }
        }
    }
}
