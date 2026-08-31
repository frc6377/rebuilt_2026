package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.NerfModeController;
import frc.robot.util.TunablePIDController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ExtenderIOSim implements ExtenderIO {

    private final SingleJointedArmSim armSim;
    private final TunablePIDController pidController;
    private final LoggedMechanism2d armMech;
    private final LoggedMechanismRoot2d armMechRoot;
    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d setpointArmLigament;

    private Angle setpoint;
    private double appliedVolts = 0.0;
    private boolean pidEnabled = true;

    private final LoggedNetworkNumber extenderStowAngle;
    private final LoggedNetworkNumber extenderIntakeAngle;
    private final LoggedNetworkNumber extenderTolerance;
    private final LoggedNetworkNumber extenderSiftAngleOne;
    private final LoggedNetworkNumber extenderSiftAngleTwo;
    private final NerfModeController nerfModeController;

    public ExtenderIOSim(NerfModeController nerfModeController) {
        this.nerfModeController = nerfModeController;
        IntakeConstants constants = nerfModeController.getIntakeConstants();
        setpoint = constants.extenderStowAngle();

        extenderStowAngle = new LoggedNetworkNumber(
                "Intake/Extender/StowAngle", constants.extenderStowAngle().in(Degrees));
        extenderIntakeAngle = new LoggedNetworkNumber(
                "Intake/Extender/IntakeAngle", constants.extenderIntakeAngle().in(Degrees));
        extenderTolerance = new LoggedNetworkNumber(
                "Intake/Extender/Tolerance", constants.extenderTolerance().in(Degrees));
        extenderSiftAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleOne", constants.extenderSiftAngleOne().in(Degrees));
        extenderSiftAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleTwo", constants.extenderSiftAngleTwo().in(Degrees));
        new LoggedNetworkNumber("Intake/Extender/DownSpeed", constants.extenderDownSpeed());

        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                constants.extenderGearing(),
                constants.extenderMOI().in(KilogramSquareMeters),
                constants.extenderArmLength().in(Meters),
                constants.extenderStowAngle().in(Radians),
                constants.extenderIntakeAngle().in(Radians),
                false,
                constants.extenderZeroAngle().in(Radians),
                0.0,
                0.0);

        pidController = new TunablePIDController(
                "Intake/ExtenderPID",
                () -> getPosition().in(Degrees),
                percent -> appliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0),
                constants.extenderConstraints());

        pidController.addPreset("default", constants.extenderNormalPID());

        armMech = new LoggedMechanism2d(5, 5);
        armMechRoot = armMech.getRoot("IntakeSimulation", 3, 3);
        armLigament = new LoggedMechanismLigament2d("arm", 2, extenderStowAngle.get(), 2.0, new Color8Bit(Color.kBlue));
        setpointArmLigament =
                new LoggedMechanismLigament2d("setpoint", 2, setpoint.in(Degrees), 1.0, new Color8Bit(Color.kRed));
        armMechRoot.append(armLigament);
        armMechRoot.append(setpointArmLigament);
    }

    private Angle getPosition() {
        return Radians.of(armSim.getAngleRads());
    }

    private boolean isAtAngle(Angle current, Angle target) {
        return Math.abs(current.minus(target).in(Degrees)) < extenderTolerance.get();
    }

    public boolean isAtAngle(Angle angle) {
        return isAtAngle(getPosition(), angle);
    }

    public void setPosition(Angle position) {
        this.setpoint = position;
        Logger.recordOutput("Intake/Extender/SetpointDegrees", this.setpoint);
        setPidEnabled(true);
        pidController.setSetpoint(position.in(Degrees));
    }

    @Override
    public void extend() {
        setPosition(Degrees.of(extenderIntakeAngle.get()));
    }

    @Override
    public void retract() {
        setPosition(Degrees.of(extenderStowAngle.get()));
    }

    @Override
    public void goToSiftAngleOne() {
        setPosition(Degrees.of(extenderSiftAngleOne.get()));
    }

    @Override
    public void goToSiftAngleTwo() {
        setPosition(Degrees.of(extenderSiftAngleTwo.get()));
    }

    @Override
    public void toggle() {
        if (isRetracted().getAsBoolean()) {
            extend();
        } else {
            retract();
        }
    }

    @Override
    public void stop() {
        pidEnabled = false;
        appliedVolts = 0.0;
        armSim.setInputVoltage(0.0);
    }

    @Override
    public void setPidEnabled(boolean enabled) {
        pidEnabled = enabled;
    }

    @Override
    public boolean isPidEnabled() {
        return pidEnabled;
    }

    @Override
    public void setMotorPercentage(double percent) {
        setPidEnabled(false);
        appliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
    }

    @Override
    public BooleanSupplier isExtended() {
        return () -> isAtAngle(Degrees.of(extenderIntakeAngle.get()));
    }

    @Override
    public BooleanSupplier isRetracted() {
        return () -> isAtAngle(Degrees.of(extenderStowAngle.get()));
    }

    @Override
    public BooleanSupplier atTarget() {
        return () -> isAtAngle(setpoint);
    }

    @Override
    public edu.wpi.first.units.measure.Current getCurrent() {
        return Amps.of(armSim.getCurrentDrawAmps());
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {

        armSim.setInputVoltage(appliedVolts);
        armSim.update(TimedRobot.kDefaultPeriod);

        Angle position = getPosition();
        inputs.position = position;
        inputs.setpoint = setpoint;
        inputs.isExtended = isAtAngle(position, Degrees.of(extenderIntakeAngle.get()));
        inputs.isRetracted = isAtAngle(position, Degrees.of(extenderStowAngle.get()));
        inputs.atTarget = isAtAngle(position, setpoint);
        inputs.velocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
        inputs.motorVoltage = Volts.of(appliedVolts);
        inputs.motorCurrent = Amps.of(armSim.getCurrentDrawAmps());
        inputs.supplyCurrent = inputs.motorCurrent;
        inputs.motorTemp = Celsius.of(25.0);

        armLigament.setAngle(getPosition());
        setpointArmLigament.setAngle(setpoint);
        Logger.recordOutput("Intake/Simulation/2D-Simulation", armMech);

        Logger.recordOutput(
                "Intake/Simulation/3D-Simulation-Pose",
                armMech.generate3dMechanism().get(0));
    }

    @Override
    public void periodic() {

        pidController.updateTunableGains();
        if (pidEnabled) {
            pidController.runPid();
        }
    }
}
