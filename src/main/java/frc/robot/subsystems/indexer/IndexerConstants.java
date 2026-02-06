package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class IndexerConstants {
    // Motor configuration
    public static final InvertedValue kIndexerDirection = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kIndexerNeutralMode = NeutralModeValue.Brake;
    public static final MotorOutputConfigs kIndexerMotorOutputConfigs =
            new MotorOutputConfigs().withInverted(kIndexerDirection).withNeutralMode(kIndexerNeutralMode);

    // Current limits
    public static final Current kStatorCurrentLimit = Amps.of(80);
    public static final CurrentLimitsConfigs kIndexerCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(false);

    // PID gains (for velocity control if needed)
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.12;

    public static final Slot0Configs kIndexerGains =
            new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV);

    public static final TalonFXConfiguration kIndexerTalonFXConfiguration = new TalonFXConfiguration()
            .withMotorOutput(kIndexerMotorOutputConfigs)
            .withSlot0(kIndexerGains)
            .withCurrentLimits(kIndexerCurrentLimitsConfigs);

    // Simulation Constants
    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.001);
    public static final double kGearRatio = 1.0;

    // Default speeds for indexer operations (duty cycle -1.0 to 1.0)
    public static final double kFeedSpeed = -0.5; // Speed to feed game pieces forward
    public static final double kReverseSpeed = 0.3; // Speed to reverse/eject

    private IndexerConstants() {}
}
