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

import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ExtenderIOReal implements ExtenderIO {

    private final @NotNull TunableTalonFX extenderMotor;
    private final @NotNull DutyCycleEncoder extenderEncoder;
    private final @NotNull TunablePIDController extenderPid;
    private boolean pidEnabled = true;
    private final @NotNull LoggedNetworkNumber extenderStowAngle;
    private final @NotNull LoggedNetworkNumber extenderIntakeAngle;
    private final @NotNull LoggedNetworkNumber extenderTolerance;
    private final @NotNull LoggedNetworkNumber extenderSiftAngleOne;
    private final @NotNull LoggedNetworkNumber extenderSiftAngleTwo;
    private final @NotNull LoggedNetworkNumber extenderCustomAngleOne;
    private final @NotNull LoggedNetworkNumber extenderCustomAngleTwo;

    public ExtenderIOReal() {

        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(ExtenderConstants.MotorConfig.kInverted)
                        .withNeutralMode(ExtenderConstants.MotorConfig.kNeutralMode))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(ExtenderConstants.MotorConfig.kRampPeriod))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(ExtenderConstants.MotorConfig.kStatorCurrentLimitExtender)
                        .withSupplyCurrentLimit(ExtenderConstants.MotorConfig.kSupplyCurrentLimitExtender));

        this.extenderMotor = new TunableTalonFX(Constants.CANIDs.MotorIDs.kExtenderMotorID, "rio", "Extender");
        this.extenderMotor.getConfigurator().apply(config);

        this.extenderEncoder = new DutyCycleEncoder(
                Constants.CANIDs.SensorIDs.kExtenderEncoderCANID,
                1.0,
                ExtenderConstants.kExtenderZeroAngle.in(Rotations));
        this.extenderEncoder.setInverted(true);

        this.extenderPid = new TunablePIDController(
                "Intake/Extender/ExtenderPID",
                () -> this.getPosition().in(Degrees),
                percent -> this.extenderMotor.set(-percent),
                ExtenderConstants.kExtenderConstraints);

        this.extenderPid.addPreset("default", ExtenderConstants.PIDF.normalPID);
        this.extenderPid.addPreset("float", ExtenderConstants.PIDF.floatPID);
        this.extenderPid.addPreset("sift", ExtenderConstants.PIDF.babyPID);

        this.extenderPid.getPIDController().enableContinuousInput(0, 359);

        this.extenderStowAngle =
                new LoggedNetworkNumber("Intake/Extender/StowAngle", ExtenderConstants.kExtenderStowAngle.in(Degrees));
        this.extenderIntakeAngle = new LoggedNetworkNumber(
                "Intake/Extender/IntakingAngle", ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        this.extenderTolerance =
                new LoggedNetworkNumber("Intake/Extender/Tolerance", ExtenderConstants.kExtenderTolerance.in(Degrees));
        this.extenderSiftAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/Sifting/SiftAngleOne", ExtenderConstants.kExtenderSiftAngleOne.in(Degrees));
        this.extenderSiftAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/Sifting/SiftAngleTwo", ExtenderConstants.kExtenderSiftAngleTwo.in(Degrees));
        this.extenderCustomAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/CustomAngleOne", ExtenderConstants.kExtenderCustomAngleOne.in(Degrees));
        this.extenderCustomAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/CustomAngleTwo", ExtenderConstants.kExtenderCustomAngleTwo.in(Degrees));

        this.extenderStowAngle.set(ExtenderConstants.kExtenderStowAngle.in(Degrees));
        this.extenderIntakeAngle.set(ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        this.extenderTolerance.set(ExtenderConstants.kExtenderTolerance.in(Degrees));
        this.extenderSiftAngleOne.set(ExtenderConstants.kExtenderSiftAngleOne.in(Degrees));
        this.extenderSiftAngleTwo.set(ExtenderConstants.kExtenderSiftAngleTwo.in(Degrees));
        this.extenderCustomAngleOne.set(ExtenderConstants.kExtenderCustomAngleOne.in(Degrees));
        this.extenderCustomAngleTwo.set(ExtenderConstants.kExtenderCustomAngleTwo.in(Degrees));
        this.extenderPid.applyPreset("default");

        this.setPidEnabled(false);
    }

    public void setPosition(@NotNull Angle position) {
        this.setPidEnabled(true);
        this.setMode(NeutralModeValue.Brake);

        this.extenderPid.setSetpoint(position.in(Degrees));
    }

    public @NotNull Angle getPosition() {
        return Degrees.of(Rotations.of(this.extenderEncoder.get()).in(Degrees));
    }

    public boolean isAtAngle(Angle angle) {
        return Math.abs((this.getPosition().minus(angle)).in(Degrees)) < this.extenderTolerance.get();
    }

    @Override
    public Current getCurrent() {
        return this.extenderMotor.getStatorCurrent().getValue();
    }

    @Override
    public void extend() {
        this.extenderPid.applyPreset("default");
        this.setPosition(Degrees.of(this.extenderIntakeAngle.get()));
    }

    @Override
    public void retract() {
        this.extenderPid.applyPreset("default");
        this.setPosition(Degrees.of(this.extenderStowAngle.get()));
    }

    @Override
    public @NotNull BooleanSupplier isExtended() {
        return () -> this.isAtAngle(Degrees.of(this.extenderIntakeAngle.get()));
    }

    @Override
    public @NotNull BooleanSupplier isRetracted() {
        return () -> this.isAtAngle(Degrees.of(this.extenderStowAngle.get()));
    }

    @Override
    public @NotNull BooleanSupplier atTarget() {
        return () -> this.isAtAngle(Degrees.of(this.extenderPid.getSetpoint()));
    }

    @Override
    public void goToSiftAngleOne() {

        this.setPosition(Degrees.of(this.extenderSiftAngleOne.get()));
    }

    @Override
    public void goToSiftAngleTwo() {

        this.setPosition(Degrees.of(this.extenderSiftAngleTwo.get()));
    }

    @Override
    public void goToCustomAngleOne() {
        this.extenderPid.applyPreset("default");
        this.setPosition(Degrees.of(this.extenderCustomAngleOne.get()));
    }

    @Override
    public void goToCustomAngleTwo() {
        this.extenderPid.applyPreset("default");
        this.setPosition(Degrees.of(this.extenderCustomAngleTwo.get()));
    }

    @Override
    public void toggleSift() {
        this.extenderPid.applyPreset("sift");
        this.extenderPid.setSpeedConstraints(ExtenderConstants.SIFT_CONSTRAINTS);
        if (Degrees.of(this.extenderPid.getSetpoint()).equals(Degrees.of(this.extenderSiftAngleOne.get()))) {
            this.goToSiftAngleTwo();
        } else {
            this.goToSiftAngleOne();
        }
    }

    @Override
    public void stop() {
        this.pidEnabled = false;
        this.extenderMotor.stopMotor();
    }

    @Override
    public void setPidEnabled(boolean enabled) {
        this.pidEnabled = enabled;
    }

    @Override
    public void setMode(NeutralModeValue mode) {
        this.extenderMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
    }

    @Override
    public void setMotorPercentage(double percent) {
        this.setPidEnabled(false);
        this.extenderMotor.set(percent);
    }

    @Override
    public void toggle() {
        this.extenderPid.applyPreset("default");
        this.extenderPid.setSpeedConstraints(ExtenderConstants.kExtenderConstraints);
        double stowDeg = this.extenderStowAngle.get();
        if (Math.abs(Degrees.of(this.extenderPid.getSetpoint())
                        .minus(Degrees.of(stowDeg))
                        .in(Degrees))
                < this.extenderTolerance.get()) {
            this.extend();
        } else {
            this.retract();
        }
    }

    @Override
    public void updateInputs(@NotNull ExtenderIOInputs inputs) {
        inputs.isExtended = this.isExtended().getAsBoolean();
        inputs.isRetracted = this.isRetracted().getAsBoolean();
        inputs.position = this.getPosition();
        inputs.setpoint = Degrees.of(this.extenderPid.getSetpoint());
        inputs.velocity = this.extenderMotor.getVelocity().getValue();
        inputs.motorVoltage = Volts.of(this.extenderMotor.getMotorVoltage().getValueAsDouble());
        inputs.motorCurrent = this.extenderMotor.getStatorCurrent().getValue();
        inputs.motorTemp = this.extenderMotor.getDeviceTemp().getValue();
        inputs.atTarget = this.atTarget().getAsBoolean();
        inputs.rawEncoderDegrees = Rotations.of(this.extenderEncoder.get()).in(Degrees);
    }

    @Override
    public void periodic() {
        this.extenderPid.updateTunableGains();
        if (this.pidEnabled) {
            this.extenderPid.runPid();
        }
        if (ExtenderConstants.floatEnabled) {
            if (this.getPosition().gte(ExtenderConstants.kExtenderFloatLimit)
                    && this.atTarget().getAsBoolean()) {
                this.extenderPid.applyPreset("float");
                this.setMode(NeutralModeValue.Coast);
            }
        }
    }
}
