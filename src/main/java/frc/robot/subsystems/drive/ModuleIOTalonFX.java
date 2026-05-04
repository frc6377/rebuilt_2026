package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Fahrenheit;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import org.jetbrains.annotations.NotNull;

public abstract class ModuleIOTalonFX implements ModuleIO {
    protected final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    protected final @NotNull TalonFX driveTalon;
    protected final @NotNull TalonFX turnTalon;
    protected final @NotNull CANcoder cancoder;

    protected final VoltageOut voltageRequest = new VoltageOut(0);
    protected final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    protected final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    protected final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    protected final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Inputs from drive motor
    protected final StatusSignal<Angle> drivePosition;
    protected final StatusSignal<AngularVelocity> driveVelocity;
    protected final StatusSignal<Voltage> driveAppliedVolts;
    protected final StatusSignal<Current> driveCurrent;

    // Inputs from turn motor
    protected final StatusSignal<Angle> turnAbsolutePosition;
    protected final StatusSignal<AngularVelocity> turnVelocity;
    protected final StatusSignal<Voltage> turnAppliedVolts;
    protected final StatusSignal<Current> turnCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

    protected ModuleIOTalonFX(
            @NotNull SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;

        this.driveTalon = new TalonFX(constants.DriveMotorId, new CANBus(TunerConstants.DrivetrainConstants.CANBusName));
        this.turnTalon = new TalonFX(constants.SteerMotorId, new CANBus(TunerConstants.DrivetrainConstants.CANBusName));
        this.cancoder = new CANcoder(constants.EncoderId, new CANBus(TunerConstants.DrivetrainConstants.CANBusName));

        // Configure drive motor
        var driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> this.driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> this.driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        var turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = constants.SteerMotorGains;
        if (Constants.Mode.SIM == Constants.currentMode)
            turnConfig.Slot0.withKD(0.5).withKS(0); // during simulation, gains are slightly different

        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        turnConfig.Feedback.FeedbackSensorSource = switch (constants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
            default -> throw new RuntimeException(
                    "You are using an unsupported swerve configuration, which this template does not support without manual customization. \n"
                            + "The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");};
        turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> this.turnTalon.getConfigurator().apply(turnConfig, 0.25));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        this.cancoder.getConfigurator().apply(cancoderConfig);

        // Create drive status signals
        this.drivePosition = this.driveTalon.getPosition();
        this.driveVelocity = this.driveTalon.getVelocity();
        this.driveAppliedVolts = this.driveTalon.getMotorVoltage();
        this.driveCurrent = this.driveTalon.getStatorCurrent();

        // Create turn status signals
        this.turnAbsolutePosition = this.cancoder.getAbsolutePosition();
        this.turnVelocity = this.turnTalon.getVelocity();
        this.turnAppliedVolts = this.turnTalon.getMotorVoltage();
        this.turnCurrent = this.turnTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(Drive.ODOMETRY_FREQUENCY, this.turnAbsolutePosition, this.drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                10.0, this.driveVelocity, this.driveAppliedVolts, this.driveCurrent, this.turnVelocity, this.turnAppliedVolts, this.turnCurrent);
        ParentDevice.optimizeBusUtilizationForAll(this.driveTalon, this.turnTalon);
    }

    @Override
    public void updateInputs(@NotNull ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(this.drivePosition, this.driveVelocity, this.driveAppliedVolts, this.driveCurrent);
        var turnStatus = BaseStatusSignal.refreshAll(this.turnVelocity, this.turnAppliedVolts, this.turnCurrent);
        var turnEncoderStatus = BaseStatusSignal.refreshAll(this.turnAbsolutePosition);

        // Update drive inputs
        inputs.driveConnected = this.driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad =
                Units.rotationsToRadians(this.drivePosition.getValueAsDouble()) / this.constants.DriveMotorGearRatio;
        inputs.driveVelocityRadPerSec =
                Units.rotationsToRadians(this.driveVelocity.getValueAsDouble()) / this.constants.DriveMotorGearRatio;
        inputs.driveAppliedVolts = this.driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = this.driveCurrent.getValueAsDouble();

        // Update turn inputs
        inputs.turnConnected = this.turnConnectedDebounce.calculate(turnStatus.isOK());
        inputs.turnEncoderConnected = this.turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(this.turnAbsolutePosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(this.turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = this.turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = this.turnCurrent.getValueAsDouble();

        if (Constants.motorTempWarningThreshold <= this.driveTalon.getDeviceTemp().getValue().in(Fahrenheit)) {
            DriverStation.reportWarning(
                    "MOTOR OVERHEATING: Drive Motor (" + this.driveTalon.getDeviceID() + ")",
                    Thread.currentThread().getStackTrace());
        }
        if (Constants.motorTempWarningThreshold <= this.turnTalon.getDeviceTemp().getValue().in(Fahrenheit)) {
            DriverStation.reportWarning(
                    "MOTOR OVERHEATING: Turn Motor (" + this.turnTalon.getDeviceID() + ")",
                    Thread.currentThread().getStackTrace());
        }
    }

    @Override
    public void setDriveOpenLoop(double output) {
        this.driveTalon.setControl(
                switch (this.constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> this.voltageRequest.withOutput(output);
                    case TorqueCurrentFOC -> this.torqueCurrentRequest.withOutput(output);
                });
    }

    @Override
    public void setTurnOpenLoop(double output) {
        this.turnTalon.setControl(
                switch (this.constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> this.voltageRequest.withOutput(output);
                    case TorqueCurrentFOC -> this.torqueCurrentRequest.withOutput(output);
                });
    }

    @Override
    public void setDriveVelocity(double wheelVelocityRadPerSec) {
        double motorVelocityRotPerSec =
                Units.radiansToRotations(wheelVelocityRadPerSec) * this.constants.DriveMotorGearRatio;
        this.driveTalon.setControl(
                switch (this.constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> this.velocityVoltageRequest.withVelocity(motorVelocityRotPerSec);
                    case TorqueCurrentFOC -> this.velocityTorqueCurrentRequest.withVelocity(motorVelocityRotPerSec);
                });
    }

    @Override
    public void setTurnPosition(@NotNull Rotation2d rotation) {
        this.turnTalon.setControl(
                switch (this.constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> this.positionVoltageRequest.withPosition(rotation.getRotations());
                    case TorqueCurrentFOC -> this.positionTorqueCurrentRequest.withPosition(rotation.getRotations());
                });
    }
}
