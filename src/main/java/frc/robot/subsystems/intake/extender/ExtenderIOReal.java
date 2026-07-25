package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.NerfModeController;
import frc.robot.util.TalonFXCurrentConfigurator;
import frc.robot.util.TalonFXNeutralModeConfigurator;
import frc.robot.util.TunablePIDController;
import frc.robot.util.TunableTalonFX;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ExtenderIOReal implements ExtenderIO {

    private final TunableTalonFX extenderMotor;
    private final DutyCycleEncoder extenderEncoder;
    private final TunablePIDController extenderPid;
    private final StatusSignal<Current> extenderSupplyCurrent;
    private final TalonFXCurrentConfigurator currentConfigurator;
    private final TalonFXNeutralModeConfigurator neutralModeConfigurator;
    private final Object configurationApplyLock = new Object();
    private boolean pidEnabled = true;
    private final LoggedNetworkNumber extenderStowAngle;
    private final LoggedNetworkNumber extenderIntakeAngle;
    private final LoggedNetworkNumber extenderTolerance;
    private final LoggedNetworkNumber extenderSiftAngleOne;
    private final LoggedNetworkNumber extenderSiftAngleTwo;
    private final LoggedNetworkNumber extenderCustomAngleOne;
    private final LoggedNetworkNumber extenderCustomAngleTwo;
    private final NerfModeController nerfModeController;

    public ExtenderIOReal(NerfModeController nerfModeController) {
        this.nerfModeController = nerfModeController;
        IntakeConstants constants = nerfModeController.getIntakeConstants();

        var currentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(constants.extenderStatorCurrentLimit())
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(constants.extenderSupplyCurrentLimit());
        var motorOutput = new MotorOutputConfigs()
                .withInverted(constants.extenderInverted())
                .withNeutralMode(constants.extenderNeutralMode());
        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(constants.extenderInverted())
                        .withNeutralMode(constants.extenderNeutralMode()))
                .withClosedLoopRamps(
                        new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(constants.extenderRampPeriod()))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(constants.extenderStatorCurrentLimit())
                        .withSupplyCurrentLimit(constants.extenderSupplyCurrentLimit()));

        extenderMotor = new TunableTalonFX(Constants.CANIDs.MotorIDs.kExtenderMotorID, "rio", "Extender");
        tryUntilOk(5, () -> extenderMotor.getConfigurator().apply(config, 0.25));
        currentConfigurator = new TalonFXCurrentConfigurator(
                "Extender", extenderMotor.getConfigurator(), currentLimits, configurationApplyLock);
        neutralModeConfigurator = new TalonFXNeutralModeConfigurator(
                "Extender", extenderMotor.getConfigurator(), motorOutput, configurationApplyLock);
        extenderSupplyCurrent = extenderMotor.getSupplyCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, extenderSupplyCurrent);

        extenderEncoder = new DutyCycleEncoder(
                Constants.CANIDs.SensorIDs.kExtenderEncoderCANID,
                1.0,
                constants.extenderZeroAngle().in(Rotations));
        extenderEncoder.setInverted(true);

        extenderPid = new TunablePIDController(
                "Intake/Extender/ExtenderPID",
                () -> getPosition().in(Degrees),
                percent -> extenderMotor.set(-percent),
                constants.extenderConstraints());

        extenderPid.addPreset("default", constants.extenderNormalPID());
        extenderPid.addPreset("float", constants.extenderFloatPID());
        extenderPid.addPreset("sift", constants.extenderBabyPID());

        extenderPid.getPIDController().enableContinuousInput(0, 359);

        extenderStowAngle = new LoggedNetworkNumber(
                "Intake/Extender/StowAngle", constants.extenderStowAngle().in(Degrees));
        extenderIntakeAngle = new LoggedNetworkNumber(
                "Intake/Extender/IntakingAngle", constants.extenderIntakeAngle().in(Degrees));
        extenderTolerance = new LoggedNetworkNumber(
                "Intake/Extender/Tolerance", constants.extenderTolerance().in(Degrees));
        extenderSiftAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/Sifting/SiftAngleOne",
                constants.extenderSiftAngleOne().in(Degrees));
        extenderSiftAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/Sifting/SiftAngleTwo",
                constants.extenderSiftAngleTwo().in(Degrees));
        extenderCustomAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/CustomAngleOne",
                constants.extenderCustomAngleOne().in(Degrees));
        extenderCustomAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/CustomAngleTwo",
                constants.extenderCustomAngleTwo().in(Degrees));

        extenderStowAngle.set(constants.extenderStowAngle().in(Degrees));
        extenderIntakeAngle.set(constants.extenderIntakeAngle().in(Degrees));
        extenderTolerance.set(constants.extenderTolerance().in(Degrees));
        extenderSiftAngleOne.set(constants.extenderSiftAngleOne().in(Degrees));
        extenderSiftAngleTwo.set(constants.extenderSiftAngleTwo().in(Degrees));
        extenderCustomAngleOne.set(constants.extenderCustomAngleOne().in(Degrees));
        extenderCustomAngleTwo.set(constants.extenderCustomAngleTwo().in(Degrees));
        extenderPid.applyPreset("default");

        setPidEnabled(false);
    }

    public void setPosition(Angle position) {
        setPidEnabled(true);
        setMode(NeutralModeValue.Brake);

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
        extenderPid.applyPreset("default");
        setPosition(Degrees.of(extenderIntakeAngle.get()));
    }

    @Override
    public void retract() {
        extenderPid.applyPreset("default");
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
        extenderPid.applyPreset("default");
        setPosition(Degrees.of(extenderCustomAngleOne.get()));
    }

    @Override
    public void goToCustomAngleTwo() {
        extenderPid.applyPreset("default");
        setPosition(Degrees.of(extenderCustomAngleTwo.get()));
    }

    @Override
    public void toggleSift() {
        extenderPid.applyPreset("sift");
        extenderPid.setSpeedConstraints(nerfModeController.getIntakeConstants().extenderSiftConstraints());
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
        neutralModeConfigurator.requestNeutralMode(mode);
    }

    @Override
    public void setMotorPercentage(double percent) {
        setPidEnabled(false);
        extenderMotor.set(percent);
    }

    @Override
    public void toggle() {
        extenderPid.applyPreset("default");
        extenderPid.setSpeedConstraints(nerfModeController.getIntakeConstants().extenderConstraints());
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
        var supplyCurrentStatus = BaseStatusSignal.refreshAll(extenderSupplyCurrent);
        logCurrentConfigurator(
                "Intake/Extender/CurrentLimit/Motor" + extenderMotor.getDeviceID(), currentConfigurator.snapshot());
        inputs.isExtended = isExtended().getAsBoolean();
        inputs.isRetracted = isRetracted().getAsBoolean();
        inputs.position = getPosition();
        inputs.setpoint = Degrees.of(extenderPid.getSetpoint());
        inputs.velocity = extenderMotor.getVelocity().getValue();
        inputs.motorVoltage = Volts.of(extenderMotor.getMotorVoltage().getValueAsDouble());
        inputs.motorCurrent = extenderMotor.getStatorCurrent().getValue();
        inputs.motorSupplyCurrent = extenderSupplyCurrent.getValue();
        inputs.motorSupplyCurrentValid = supplyCurrentStatus.isOK();
        inputs.motorTemp = extenderMotor.getDeviceTemp().getValue();
        inputs.atTarget = atTarget().getAsBoolean();
        inputs.rawEncoderDegrees = Rotations.of(extenderEncoder.get()).in(Degrees);
        logNeutralModeConfigurator("Intake/Extender/NeutralMode", neutralModeConfigurator.snapshot());
    }

    @Override
    public void periodic() {
        extenderPid.updateTunableGains();
        if (pidEnabled) {
            extenderPid.runPid();
        }
        if (nerfModeController.getIntakeConstants().extenderFloatEnabled()) {
            if (getPosition().gte(nerfModeController.getIntakeConstants().extenderFloatLimit())
                    && atTarget().getAsBoolean()) {
                extenderPid.applyPreset("float");
                setMode(NeutralModeValue.Coast);
            }
        }
    }

    @Override
    public void setSupplyCurrentLimit(double currentLimitAmps) {
        currentConfigurator.requestSupplyCurrentLimit(currentLimitAmps);
    }

    private static void logCurrentConfigurator(String key, TalonFXCurrentConfigurator.Snapshot snapshot) {
        Logger.recordOutput(key + "/RequestedLimitAmps", snapshot.requestedLimitAmps());
        Logger.recordOutput(key + "/LastSuccessfulAcknowledgedLimitAmps", snapshot.lastSuccessfulLimitAmps());
        Logger.recordOutput(key + "/RequestedRevision", snapshot.requestedRevision());
        Logger.recordOutput(key + "/LastSuccessfulAcknowledgedRevision", snapshot.lastSuccessfulRevision());
        Logger.recordOutput(key + "/LastOutcome", snapshot.lastOutcome().name());
        Logger.recordOutput(key + "/LastStatusName", snapshot.lastStatusName());
        Logger.recordOutput(key + "/LastStatusDescription", snapshot.lastStatusDescription());
        Logger.recordOutput(key + "/LastException", snapshot.lastException());
        Logger.recordOutput(key + "/LastAttemptAgeSeconds", snapshot.lastAttemptAgeSeconds());
        Logger.recordOutput(
                key + "/LastSuccessfulAcknowledgedApplyAgeSeconds", snapshot.lastSuccessfulApplyAgeSeconds());
        Logger.recordOutput(key + "/AttemptCount", snapshot.attemptCount());
        Logger.recordOutput(key + "/SuccessCount", snapshot.successCount());
        Logger.recordOutput(key + "/FailureCount", snapshot.failureCount());
        Logger.recordOutput(key + "/ExceptionCount", snapshot.exceptionCount());
        Logger.recordOutput(key + "/RetryAttemptCount", snapshot.retryAttemptCount());
        Logger.recordOutput(key + "/DeduplicatedRequestCount", snapshot.deduplicatedRequestCount());
        Logger.recordOutput(key + "/Pending", snapshot.pending());
        Logger.recordOutput(key + "/Retrying", snapshot.retrying());
        Logger.recordOutput(key + "/InFlight", snapshot.inFlight());
        Logger.recordOutput(key + "/Closed", snapshot.closed());
        Logger.recordOutput(key + "/WorkerAlive", snapshot.workerAlive());
    }

    private static void logNeutralModeConfigurator(String key, TalonFXNeutralModeConfigurator.Snapshot snapshot) {
        Logger.recordOutput(key + "/Requested", snapshot.requestedMode().name());
        Logger.recordOutput(
                key + "/LastSuccessfulAcknowledged",
                snapshot.lastSuccessfulMode() == null
                        ? "Unacknowledged"
                        : snapshot.lastSuccessfulMode().name());
        Logger.recordOutput(key + "/RequestedRevision", snapshot.requestedRevision());
        Logger.recordOutput(key + "/LastSuccessfulAcknowledgedRevision", snapshot.lastSuccessfulRevision());
        Logger.recordOutput(key + "/LastStatus", snapshot.lastStatusName());
        Logger.recordOutput(key + "/LastException", snapshot.lastException());
        Logger.recordOutput(key + "/AttemptCount", snapshot.attemptCount());
        Logger.recordOutput(key + "/SuccessCount", snapshot.successCount());
        Logger.recordOutput(key + "/FailureCount", snapshot.failureCount());
        Logger.recordOutput(key + "/Pending", snapshot.pending());
        Logger.recordOutput(key + "/Retrying", snapshot.retrying());
        Logger.recordOutput(key + "/InFlight", snapshot.inFlight());
        Logger.recordOutput(key + "/WorkerAlive", snapshot.workerAlive());
    }
}
