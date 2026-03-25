package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.CANIDs.MotorIDs;
import frc.robot.util.TunableTalonFX;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClimberIOSim implements ClimberIO {

    private final TunableTalonFX pivotMotor;
    private final TunableTalonFX hookMotor;

    private final SingleJointedArmSim pivotSim;

    private final DCMotorSim hookSim;

    private static LoggedMechanism2d climbMech;
    private LoggedMechanismRoot2d mechRoot;
    private LoggedMechanismLigament2d pivotLig;
    private LoggedMechanismLigament2d hookSpinIndicator;

    private double pivotAppliedVolts = 0.0;
    private double hookAppliedVolts = 0.0;

    private double hookRunningSeconds = 0.0;
    private boolean hookContactSimulated = false;
    private boolean climbDoneSimulated = false;

    private static final double kHookContactSimDelay = 1.5;
    private static final double kClimbDoneSimDelay = 3.0;

    public ClimberIOSim() {
        pivotMotor = new TunableTalonFX(MotorIDs.kClimbMotor2ID, "sim", "PivotMotor");
        hookMotor = new TunableTalonFX(MotorIDs.kClimbMotor1ID, "sim", "HookMotor");

        tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(ClimbConstants.kPivotMotorConfigSim, 0.25));
        tryUntilOk(5, () -> hookMotor.getConfigurator().apply(ClimbConstants.kHookMotorConfigSim, 0.25));

        pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                ClimbConstants.kPivotGearRatio,
                SingleJointedArmSim.estimateMOI(ClimbConstants.kPivotArmLengthMeters, ClimbConstants.kPivotArmMassKg),
                ClimbConstants.kPivotArmLengthMeters,
                ClimbConstants.kPivotMinAngleRad, // stow angle
                ClimbConstants.kPivotMaxAngleRad, // extended angle
                true, // simulate gravity
                ClimbConstants.kPivotMinAngleRad); // start at stow

        hookSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1), ClimbConstants.kHookMomentOfInertia, ClimbConstants.kHookGearRatio),
                DCMotor.getKrakenX60(1));

        climbMech = new LoggedMechanism2d(2, 2);
        mechRoot = climbMech.getRoot("ClimberRoot", 1.0, 0.1);

        pivotLig = new LoggedMechanismLigament2d(
                "PivotArm",
                ClimbConstants.kPivotArmLengthMeters,
                Units.radiansToDegrees(ClimbConstants.kPivotMinAngleRad),
                6,
                new Color8Bit(Color.kOrange));

        hookSpinIndicator = new LoggedMechanismLigament2d("HookSpin", 0.1, 0, 3, new Color8Bit(Color.kAqua));

        mechRoot.append(pivotLig);
        pivotLig.append(hookSpinIndicator);

        SmartDashboard.putData("Mech2Ds/ClimberMech", climbMech);
    }

    @Override
    public void goToPivotAngle(Angle angle) {
        pivotMotor.setControl(new MotionMagicVoltage(angle.in(Rotations)).withSlot(0));
    }

    @Override
    public void setHookPercent(double percent) {
        hookMotor.set(percent);
        if (Math.abs(percent) < 0.05) {
            hookRunningSeconds = 0.0;
            hookContactSimulated = false;
            climbDoneSimulated = false;
        }
    }

    @Override
    public void stop() {
        pivotMotor.stopMotor();
        hookMotor.stopMotor();
        pivotAppliedVolts = 0.0;
        hookAppliedVolts = 0.0;
        hookRunningSeconds = 0.0;
        hookContactSimulated = false;
        climbDoneSimulated = false;
    }

    @Override
    public void set(double percent) {
        pivotMotor.set(percent);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.pivotAngle = Rotations.of(pivotSim.getAngleRads() / (2 * Math.PI));
        inputs.pivotAppliedVoltage = Volts.of(pivotAppliedVolts);
        inputs.pivotStatorCurrent = Amps.of(pivotSim.getCurrentDrawAmps());
        inputs.pivotSupplyCurrent = Amps.of(pivotSim.getCurrentDrawAmps());
        inputs.pivotTemperatureCelsius = 25.0;
        inputs.pivotMotorConnected = true;
        inputs.pivotAbsoluteEncoderPosition = (pivotSim.getAngleRads() / (2 * Math.PI)) % 1.0;

        inputs.hookAngle = Rotations.of(hookSim.getAngularPositionRotations());
        inputs.hookAppliedVoltage = Volts.of(hookAppliedVolts);
        inputs.hookTemperatureCelsius = 25.0;
        inputs.hookMotorConnected = true;
        inputs.hookAbsoluteEncoderPosition = hookSim.getAngularPositionRotations() % 1.0;

        if (hookContactSimulated && !climbDoneSimulated && hookRunningSeconds >= kClimbDoneSimDelay) {
            inputs.hookStatorCurrent = Amps.of(ClimbConstants.kClimbDoneAmps + 5.0);
            inputs.hookSupplyCurrent = inputs.hookStatorCurrent;
            climbDoneSimulated = true;
        } else if (!hookContactSimulated && hookRunningSeconds >= kHookContactSimDelay) {
            inputs.hookStatorCurrent = Amps.of(ClimbConstants.kHookContactAmps + 5.0);
            inputs.hookSupplyCurrent = inputs.hookStatorCurrent;
            hookContactSimulated = true;
        } else {
            inputs.hookStatorCurrent = Amps.of(hookSim.getCurrentDrawAmps());
            inputs.hookSupplyCurrent = inputs.hookStatorCurrent;
        }
    }

    @Override
    public Angle getPivotAngle() {
        return Rotations.of(pivotSim.getAngleRads() / (2 * Math.PI));
    }

    @Override
    public void resetToAbsolute() {
        // Sim encoder is always accurate — nothing to reset
    }

    @Override
    public void zeroEncoder() {
        // No-op for sim
    }

    @Override
    public void periodic() {
        pivotAppliedVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
        hookAppliedVolts = hookMotor.getMotorVoltage().getValueAsDouble();

        pivotSim.setInputVoltage(pivotAppliedVolts);
        hookSim.setInputVoltage(hookAppliedVolts);

        pivotSim.update(TimedRobot.kDefaultPeriod);
        hookSim.update(TimedRobot.kDefaultPeriod);

        if (Math.abs(hookAppliedVolts) > 0.5) {
            hookRunningSeconds += TimedRobot.kDefaultPeriod;
        }

        pivotLig.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));
        hookSpinIndicator.setAngle(Units.radiansToDegrees(hookSim.getAngularPositionRotations() * 2 * Math.PI));

        Logger.recordOutput("Climb/Sim/Pivot/AngleRad", pivotSim.getAngleRads());
        Logger.recordOutput("Climb/Sim/Pivot/VelocityRadS", pivotSim.getVelocityRadPerSec());
        Logger.recordOutput("Climb/Sim/Pivot/CurrentAmps", pivotSim.getCurrentDrawAmps());
        Logger.recordOutput("Climb/Sim/Pivot/AppliedVolts", pivotAppliedVolts);

        Logger.recordOutput("Climb/Sim/Hook/PositionRot", hookSim.getAngularPositionRotations());
        Logger.recordOutput("Climb/Sim/Hook/VelocityRPS", hookSim.getAngularVelocityRPM() / 60.0);
        Logger.recordOutput("Climb/Sim/Hook/CurrentAmps", hookSim.getCurrentDrawAmps());
        Logger.recordOutput("Climb/Sim/Hook/AppliedVolts", hookAppliedVolts);
        Logger.recordOutput("Climb/Sim/Hook/RunningSeconds", hookRunningSeconds);
        Logger.recordOutput("Climb/Sim/Hook/ContactSimulated", hookContactSimulated);
        Logger.recordOutput("Climb/Sim/Hook/ClimbDoneSimulated", climbDoneSimulated);

        Logger.recordOutput("Climb/Sim/2DMech", climbMech);

        pivotMotor.updateTunableGains();
        hookMotor.updateTunableGains();
    }
}
