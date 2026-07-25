package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterConfig;
import frc.robot.util.TunablePIDController.PIDConfig;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public record IntakeConstants(
        Distance kIntakeWidth,
        Distance kIntakeExtension,
        IntakeSide kIntakeSide,
        int kIntakeCapacity,
        // Roller
        boolean rollerFollowerEnabled,
        boolean rollerIdleEnabled,
        double rollerIntakePercent,
        double rollerOuttakePercent,
        double rollerIdlePercent,
        AngularVelocity rollerIntakeSpeed,
        AngularVelocity rollerOuttakeSpeed,
        AngularVelocity rollerIdleSpeed,
        AngularVelocity rollerPowerManagementActiveThreshold,
        double rollerKP,
        double rollerKI,
        double rollerKD,
        double rollerKS,
        double rollerKV,
        double rollerKA,
        double rollerRampPeriod,
        Current rollerStatorCurrentLimit,
        Current rollerSupplyCurrentLimit,
        InvertedValue rollerInverted,
        NeutralModeValue rollerNeutralMode,
        MotorAlignmentValue rollerFollowerInverted,
        ShooterConfig rollerConfig,
        // Extender
        boolean extenderFloatEnabled,
        double extenderGearing,
        MomentOfInertia extenderMOI,
        Distance extenderArmLength,
        double extenderDownSpeed,
        Angle extenderStowAngle,
        Angle extenderIntakeAngle,
        Angle extenderTolerance,
        Angle extenderSiftAngleOne,
        Angle extenderSiftAngleTwo,
        Angle extenderCustomAngleOne,
        Angle extenderCustomAngleTwo,
        Angle extenderFloatLimit,
        Angle extenderZeroAngle,
        TrapezoidProfile.Constraints extenderConstraints,
        TrapezoidProfile.Constraints extenderSiftConstraints,
        Time extenderSiftTimeout,
        PIDConfig extenderNormalPID,
        PIDConfig extenderFloatPID,
        PIDConfig extenderBabyPID,
        double extenderRampPeriod,
        Current extenderStatorCurrentLimit,
        Current extenderSupplyCurrentLimit,
        InvertedValue extenderInverted,
        NeutralModeValue extenderNeutralMode) {}
