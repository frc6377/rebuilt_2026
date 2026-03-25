package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.CANIDs.MotorIDs;
import frc.robot.Constants.CANIDs.SensorIDs;
import frc.robot.util.TunableTalonFX;
import org.littletonrobotics.junction.Logger;

public class ClimberIOReal implements ClimberIO {

    // Motor 1 — hook (top of rail, rotates continuously onto pole and lifts robot)
    // Motor 2 — pivot (swings arm from stow to pole position)
    final TunableTalonFX hookMotor;
    final TunableTalonFX pivotMotor;

    // Through bore encoders — absolute position fed into TalonFX on enable
    // so neither motor needs a homing move
    final DutyCycleEncoder hookEncoder;
    final DutyCycleEncoder pivotEncoder;

    // MotionMagic is only used on the pivot — the hook just runs at percent output
    final MotionMagicVoltage pivotMotionMagicRequest;

    public ClimberIOReal() {
        hookMotor = new TunableTalonFX(MotorIDs.kClimbMotor1ID, "rio", "HookMotor");
        pivotMotor = new TunableTalonFX(MotorIDs.kClimbMotor2ID, "rio", "PivotMotor");

        hookEncoder = new DutyCycleEncoder(SensorIDs.kClimbHookEncoderID);
        pivotEncoder = new DutyCycleEncoder(SensorIDs.kClimbPivotEncoderID);

        // Apply configs
        tryUntilOk(5, () -> hookMotor.getConfigurator().apply(ClimbConstants.kHookMotorConfigReal, 0.25));
        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(ClimbConstants.kPivotMotorConfigReal, 0.25));

        // Seed both TalonFX position registers from the absolute through bore encoders
        // This means true angle is known immediately — no homing move needed
        hookMotor.setPosition(hookEncoder.get());
        pivotMotor.setPosition(pivotEncoder.get());

        // Apply MotionMagic configs to pivot only
        Slot0Configs pivotPID = new Slot0Configs();
        pivotPID.kP = ClimbConstants.PIDF.kP;
        pivotPID.kI = ClimbConstants.PIDF.kI;
        pivotPID.kD = ClimbConstants.PIDF.kD;
        pivotPID.kS = ClimbConstants.PIDF.kS;
        pivotPID.kV = ClimbConstants.PIDF.kV;
        pivotPID.kA = ClimbConstants.PIDF.kA;

        MotionMagicConfigs pivotMotionMagic = new MotionMagicConfigs();
        pivotMotionMagic.MotionMagicCruiseVelocity = ClimbConstants.MotionMagic.kCruiseVelocity;
        pivotMotionMagic.MotionMagicAcceleration = ClimbConstants.MotionMagic.kAcceleration;
        pivotMotionMagic.MotionMagicJerk = ClimbConstants.MotionMagic.kJerk;

        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotPID, 0.25));
        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotMotionMagic, 0.25));

        pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    }

    @Override
    public void goToPivotAngle(Angle angle) {
        Logger.recordOutput("Climb/Real/Pivot/Setpoint (Rotations)", angle.in(Rotations));
        pivotMotor.setControl(pivotMotionMagicRequest.withPosition(angle.in(Rotations)));
    }

    @Override
    public void setHookPercent(double percent) {
        hookMotor.set(percent);
    }

    @Override
    public void stop() {
        hookMotor.stopMotor();
        pivotMotor.stopMotor();
    }

    @Override
    public void set(double percent) {
        pivotMotor.set(percent);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.pivotAngle = Rotations.of(pivotMotor.getPosition().getValue().in(Rotations));
        inputs.pivotAppliedVoltage = Volts.of(pivotMotor.getMotorVoltage().getValueAsDouble());
        inputs.pivotStatorCurrent = Amps.of(pivotMotor.getStatorCurrent().getValueAsDouble());
        inputs.pivotSupplyCurrent = Amps.of(pivotMotor.getSupplyCurrent().getValueAsDouble());
        inputs.pivotTemperatureCelsius = pivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.pivotMotorConnected = pivotMotor.isAlive();

        inputs.pivotAbsoluteEncoderPosition = pivotEncoder.get();

        inputs.hookAngle = Rotations.of(hookMotor.getPosition().getValue().in(Rotations));
        inputs.hookAppliedVoltage = Volts.of(hookMotor.getMotorVoltage().getValueAsDouble());
        inputs.hookStatorCurrent = Amps.of(hookMotor.getStatorCurrent().getValueAsDouble());
        inputs.hookSupplyCurrent = Amps.of(hookMotor.getSupplyCurrent().getValueAsDouble());
        inputs.hookTemperatureCelsius = hookMotor.getDeviceTemp().getValueAsDouble();
        inputs.hookMotorConnected = hookMotor.isAlive();
        inputs.hookAbsoluteEncoderPosition = hookEncoder.get();
    }

    @Override
    public Angle getPivotAngle() {
        return Rotations.of(pivotMotor.getPosition().getValue().in(Rotations));
    }

    @Override
    public void resetToAbsolute() {
        hookMotor.setPosition(hookEncoder.get());
        pivotMotor.setPosition(pivotEncoder.get());
    }

    @Override
    public void zeroEncoder() {
        hookMotor.setPosition(0);
        pivotMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        hookMotor.updateTunableGains();
        pivotMotor.updateTunableGains();

        Logger.recordOutput(
                "Climb/Real/Pivot/Angle (Rotations)", getPivotAngle().in(Rotations));
        Logger.recordOutput("Climb/Real/Pivot/AbsEncoder", pivotEncoder.get());
        Logger.recordOutput(
                "Climb/Real/Hook/Angle (Rotations)",
                hookMotor.getPosition().getValue().in(Rotations));
        Logger.recordOutput("Climb/Real/Hook/AbsEncoder", hookEncoder.get());
        Logger.recordOutput(
                "Climb/Real/Hook/StatorCurrent (A)",
                hookMotor.getStatorCurrent().getValueAsDouble());
    }
}
