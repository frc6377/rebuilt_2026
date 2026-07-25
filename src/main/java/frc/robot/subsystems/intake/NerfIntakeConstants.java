package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class NerfIntakeConstants {
    public static final IntakeConstants DATA;

    static {
        IntakeConstants base = BaseIntakeConstants.DATA;

        DATA = new IntakeConstants(
                base.kIntakeWidth(),
                base.kIntakeExtension(),
                base.kIntakeSide(),
                base.kIntakeCapacity(),
                base.rollerFollowerEnabled(),
                base.rollerIdleEnabled(),
                base.rollerIntakePercent(),
                base.rollerOuttakePercent(),
                base.rollerIdlePercent(),
                base.rollerPowerManagementActiveThreshold(),
                base.rollerIntakeSpeed(),
                base.rollerOuttakeSpeed(),
                base.rollerIdleSpeed(),
                base.rollerKP(),
                base.rollerKI(),
                base.rollerKD(),
                base.rollerKS(),
                base.rollerKV(),
                base.rollerKA(),
                base.rollerRampPeriod(),
                base.rollerStatorCurrentLimit(),
                base.rollerSupplyCurrentLimit(),
                base.rollerInverted(),
                base.rollerNeutralMode(),
                base.rollerFollowerInverted(),
                base.rollerConfig(),
                base.extenderFloatEnabled(),
                base.extenderGearing(),
                base.extenderMOI(),
                base.extenderArmLength(),
                base.extenderDownSpeed(),
                base.extenderStowAngle(),
                base.extenderIntakeAngle(),
                base.extenderTolerance(),
                base.extenderSiftAngleOne(),
                base.extenderSiftAngleTwo(),
                base.extenderCustomAngleOne(),
                base.extenderCustomAngleTwo(),
                base.extenderFloatLimit(),
                base.extenderZeroAngle(),
                new TrapezoidProfile.Constraints(50000, 3750), // extenderConstraints (nerfed from 100000, 7500)
                new TrapezoidProfile.Constraints(50000, 3750), // extenderSiftConstraints (nerfed from 100000, 7500)
                base.extenderSiftTimeout(),
                base.extenderNormalPID(),
                base.extenderFloatPID(),
                base.extenderBabyPID(),
                base.extenderRampPeriod(),
                base.extenderStatorCurrentLimit(),
                base.extenderSupplyCurrentLimit(),
                base.extenderInverted(),
                base.extenderNeutralMode());
    }
}
