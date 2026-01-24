package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;

public final class ShooterConstants {
    public static final InvertedValue kShooterOuttakeDirection = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kShooterNeutralMode = NeutralModeValue.Coast;
    public static final MotorOutputConfigs kShooterMotorOutputConfigs =
            new MotorOutputConfigs().withInverted(kShooterOuttakeDirection).withNeutralMode(kShooterNeutralMode);
    public static final CurrentLimitsConfigs kShooterCurrentLimitsConfigs =
            new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(40));
    // PID gains for velocity control (Slot0)
    // kP: Proportional gain - output per unit of error in velocity
    // kI: Integral gain - output per unit of integrated error
    // kD: Derivative gain - output per unit of error derivative
    // kS: Static feedforward - output to overcome static friction
    // kV: Velocity feedforward - output per unit of requested velocity
    public static final Slot0Configs kShooterGains =
            new Slot0Configs().withKP(0.1).withKI(0).withKD(0).withKS(0).withKV(0.12);
    public static final PIDController kShooterSimPIDController = new PIDController(0.1, 0.0, 0.0);
    public static final TalonFXConfiguration kShooterTalonFXConfiguration = new TalonFXConfiguration()
            .withMotorOutput(kShooterMotorOutputConfigs)
            .withSlot0(kShooterGains)
            .withCurrentLimits(kShooterCurrentLimitsConfigs);
    public static final Distance kShooterWheelDiameter = Centimeters.of(10); // 10 cm diameter FIXME
    public static final double kShooterEfficiency = 0.85; // 85% efficiency
    // Simulation Constants
    public static final double kMOI = 0.01;
    public static final double kGearRatio = 1.0; // FIXME
    public static final int kMotorCount = 1;
}
