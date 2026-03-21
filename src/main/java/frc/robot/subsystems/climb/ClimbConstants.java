package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.EnabledSubsystems;

public class ClimbConstants {
    public class PIDF {
        public static final double kP = 20;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
    }

    // Sim PID
    public class SimPIDF {
        public static final double kP = 1000;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
    }

    public static class MotionMagic {
        public static final double kCruiseVelocity = 1500;
        public static final double kAcceleration = 3000;
        public static final double kJerk = 1;
    }

    public static final boolean kDisabled = !EnabledSubsystems.kClimb;

    public static final TalonFXConfiguration kPivotMotorConfigSim = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs()
                    .withKP(SimPIDF.kP)
                    .withKI(SimPIDF.kI)
                    .withKD(SimPIDF.kD)
                    .withKS(0)
                    .withKV(0)
                    .withKA(0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(70)));

    public static final TalonFXConfiguration kHookMotorConfigSim = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs()
                    .withKP(SimPIDF.kP)
                    .withKI(SimPIDF.kI)
                    .withKD(SimPIDF.kD)
                    .withKS(0)
                    .withKV(0)
                    .withKA(0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(70)));

    public static final TalonFXConfiguration kPivotMotorConfigReal = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs()
                    .withKP(PIDF.kP)
                    .withKI(PIDF.kI)
                    .withKD(PIDF.kD)
                    .withKS(PIDF.kS)
                    .withKV(PIDF.kV)
                    .withKA(PIDF.kA))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(70)));

        public static final TalonFXConfiguration kHookMotorConfigReal = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs()
                    .withKP(PIDF.kP)
                    .withKI(PIDF.kI)
                    .withKD(PIDF.kD)
                    .withKS(PIDF.kS)
                    .withKV(PIDF.kV)
                    .withKA(PIDF.kA))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(70)));

    public static final Distance kSetpointTolerance = Inches.of(0.5);

    // Sim Constants
    public static final DCMotor kClimbGearBox = DCMotor.getFalcon500(2);
    public static final double kPivotGearRatio = 135;
    public static final double kPivotArmLengthMeters = Inches.of(12).in(Meters);
    public static final double kPivotArmMassKg = Pounds.of(5).in(Kilograms);
    public static final double kPivotMinAngleRad = 0;
    public static final double kPivotMaxAngleRad = Math.PI / 2;

    // Hook Sim Constants (placeholders, need to be tuned)
    public static final double kHookGearRatio = 10;
    public static final double kHookMomentOfInertia = 0.0; // Placeholder value

    public static final double kAppliedVolts = 0.0;
    public static final boolean kClosedLoopControl = false;

    public static final double kClimbDoneAmps = 10;
    public static final double kHookContactAmps = 0.5;
}
