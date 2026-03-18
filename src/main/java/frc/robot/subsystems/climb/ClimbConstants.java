package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

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

    public static final boolean kDisabled = false;

    public static final TalonFXConfiguration kClimbMotorConfigReal = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
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

    public static final TalonFXConfiguration kClimbMotorConfigSim = new TalonFXConfiguration()
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

    public static final Distance kSetpointTolerance = Inches.of(0.5);

    // Sim Constants
    public static final DCMotor kClimbGearBox = DCMotor.getKrakenX60(1);
    public static final double kClimbGearRatio = 9 * 5 * 3;
    public static final Mass kCarriageMass = Pounds.of(2);
    public static final Distance kElevatorDrumRadius = Inches.of(0.8596);
    public static final Distance kElevatorDrumCircumference =
            kElevatorDrumRadius.times(2).times(Math.PI);
    public static final Distance kClimbMinHeight = Inches.of(0);
    public static final Distance kClimbMaxHeight = Inches.of(30);
    public static final boolean kSimulateGravity = true;
    public static final Distance kStartHeight = Inches.of(0);
    public static final double kClimbSpeed = 0.01;
    public static final int kLimitSwitchPort = 0;

    public static final double kAppliedVolts = 0.0;
    public static final boolean kClosedLoopControl = false;
    //     public static final double kIntegralAccumulator = 0.0;
    //     public static final double kPreviousError = 0.0;
    //     public static final double kDT = 0.02;

    public static final SoftwareLimitSwitchConfigs kLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ClimbConstants.kClimbMaxHeight
                    .times(ClimbConstants.kClimbGearRatio)
                    .div(ClimbConstants.kElevatorDrumCircumference)
                    .times(Rotations.one()))
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ClimbConstants.kClimbMinHeight
                    .times(ClimbConstants.kClimbGearRatio)
                    .div(ClimbConstants.kElevatorDrumCircumference)
                    .times(Rotations.of(1)));
}
