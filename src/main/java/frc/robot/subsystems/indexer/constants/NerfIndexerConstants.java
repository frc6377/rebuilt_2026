package frc.robot.subsystems.indexer.constants;

public class NerfIndexerConstants {
    public static final IndexerConstants DATA = new IndexerConstants(
            BaseIndexerConstants.DATA.kCollectorSpeed(),
            BaseIndexerConstants.DATA.kCollectorVariableSpeed(),
            BaseIndexerConstants.DATA.kCollectorRPM(),
            BaseIndexerConstants.DATA.canBus(),
            BaseIndexerConstants.DATA.voltageClosedLoopRampPeriod(),
            BaseIndexerConstants.DATA.peakForwardTorqueCurrent(),
            BaseIndexerConstants.DATA.peakReverseTorqueCurrent(),
            BaseIndexerConstants.DATA.motorInverted(),
            BaseIndexerConstants.DATA.motorNeutralMode(),
            BaseIndexerConstants.DATA.kStatorCurrentLimit(),
            BaseIndexerConstants.DATA.kSupplyCurrentLimit(),
            BaseIndexerConstants.DATA.kP(),
            BaseIndexerConstants.DATA.kI(),
            BaseIndexerConstants.DATA.kD(),
            BaseIndexerConstants.DATA.simKP(),
            BaseIndexerConstants.DATA.simKI(),
            BaseIndexerConstants.DATA.simKD(),
            BaseIndexerConstants.DATA.nominalVoltage(),
            BaseIndexerConstants.DATA.kS(),
            BaseIndexerConstants.DATA.kA(),
            BaseIndexerConstants.DATA.rollerMassKg(),
            BaseIndexerConstants.DATA.rollerRadiusM(),
            BaseIndexerConstants.DATA.rollerGearing());
}
