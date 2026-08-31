package frc.robot.subsystems.indexer.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class BaseIndexerConstants {
    public static final IndexerConstants DATA = new IndexerConstants(
            -0.5, // kCollectorSpeed
            -.15, // kCollectorVariableSpeed
            RPM.of(135), // kCollectorRPM
            "rio", // canBus
            0.02, // voltageClosedLoopRampPeriod
            40.0, // peakForwardTorqueCurrent
            40.0, // peakReverseTorqueCurrent
            InvertedValue.Clockwise_Positive, // motorInverted
            NeutralModeValue.Brake, // motorNeutralMode
            Amps.of(25), // kStatorCurrentLimit
            Amps.of(25), // kSupplyCurrentLimit
            0.0, // kP
            0.0, // kI
            0.0, // kD
            0.1, // simKP
            0.0, // simKI
            0.0, // simKD
            12.0, // nominalVoltage (using a default value instead of RobotController.getBatteryVoltage() for
            // consistency)
            0.0, // kS
            0.0, // kA
            0.25, // rollerMassKg
            0.02, // rollerRadiusM
            1.0 // rollerGearing
            );
}
