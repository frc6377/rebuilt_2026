package frc.robot.subsystems.indexer.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public record IndexerConstants(
        double kCollectorSpeed,
        double kCollectorVariableSpeed,
        AngularVelocity kCollectorRPM,
        String canBus,
        double voltageClosedLoopRampPeriod,
        double peakForwardTorqueCurrent,
        double peakReverseTorqueCurrent,
        InvertedValue motorInverted,
        NeutralModeValue motorNeutralMode,
        Current kStatorCurrentLimit,
        Current kSupplyCurrentLimit,
        double kP,
        double kI,
        double kD,
        double simKP,
        double simKI,
        double simKD,
        double nominalVoltage,
        double kS,
        double kA,
        double rollerMassKg,
        double rollerRadiusM,
        double rollerGearing) {
    public double rollerMOI() {
        return 0.5 * rollerMassKg * rollerRadiusM * rollerRadiusM;
    }
}
