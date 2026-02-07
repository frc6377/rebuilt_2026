package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ClimbConstants {
    public static final TalonFXConfiguration kClimbMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs()
                    .withKP(1)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0)
                    .withKV(0)
                    .withKA(0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(Amps.of(70)));

    // Sim Constants
    public static final DCMotor kClimbGearBox = DCMotor.getKrakenX60(2);
    public static final double kClimbGearRatio = 36;
    public static final Mass kCarriageMass = Pounds.of(4.75);
    public static final Distance kElevatorDrumRadius = Inches.of(1.708);
    public static final Distance kElevatorDrumCircumference =
            kElevatorDrumRadius.times(2).times(Math.PI);
    public static final Distance kClimbMinHeight = Inches.zero(); // inches
    public static final Distance kClimbMaxHeight = Inches.of(30); // inches
    public static final boolean kSimulateGravity = true;
    public static final Distance kStartHeight = Inches.zero();
    public static final double kClimbSpeed = 0.5;

    public class PIDF {
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
    }
}
