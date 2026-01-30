package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Percent;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterConstants {
    public static final InvertedValue kShooterOuttakeDirection = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kShooterNeutralMode = NeutralModeValue.Coast;
    public static final MotorOutputConfigs kShooterMotorOutputConfigs =
            new MotorOutputConfigs().withInverted(kShooterOuttakeDirection).withNeutralMode(kShooterNeutralMode);

    // Current limits
    public static final Current kStatorCurrentLimit = Amps.of(80.0);
    public static final CurrentLimitsConfigs kShooterCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(false);

    // PID/FF gains for velocity control (Slot0) - Real Robot
    // kP: Proportional gain - output per unit of error in velocity
    // kI: Integral gain - output per unit of integrated error
    // kD: Derivative gain - output per unit of error derivative
    // kS: Static feedforward - output to overcome static friction
    // kV: Velocity feedforward - output per unit of requested velocity
    public static final double kP = 0.75;
    public static final double kI = 0.0;
    public static final double kD = 0;
    public static final double kS = 0.0;
    public static final double kV = 0.12;

    public static final Slot0Configs kShooterGains =
            new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV);

    // PID gains for simulation
    public static final double kSimP = 0.1;
    public static final double kSimI = 0.0;
    public static final double kSimD = 0.0;

    public static final PIDController kShooterSimPIDController = new PIDController(kSimP, kSimI, kSimD);

    public static final TalonFXConfiguration kShooterTalonFXConfiguration = new TalonFXConfiguration()
            .withMotorOutput(kShooterMotorOutputConfigs)
            .withSlot0(kShooterGains)
            .withCurrentLimits(kShooterCurrentLimitsConfigs);
    public static final Distance kShooterWheelDiameter = Centimeters.of(10); // 10 cm diameter FIXME
    public static final Dimensionless kShooterEfficiency = Percent.of(85); // 85% efficiency

    // Simulation Constants
    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.01);
    public static final double kGearRatio = 1.0; // FIXME
    public static final int kMotorCount = 1;
}
