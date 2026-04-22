package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunablePIDController;
import java.util.function.BooleanSupplier;

import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ExtenderIOSim implements ExtenderIO {

    private final @NotNull SingleJointedArmSim armSim;
    private final @NotNull TunablePIDController pidController;
    private final @NotNull LoggedMechanism2d armMech;
    private final @NotNull LoggedMechanismLigament2d armLigament;
    private final @NotNull LoggedMechanismLigament2d setpointArmLigament;

    private Angle setpoint;
    private double appliedVolts = 0.0;
    private boolean pidEnabled = true;

    private final @NotNull LoggedNetworkNumber extenderStowAngle;
    private final @NotNull LoggedNetworkNumber extenderIntakeAngle;
    private final @NotNull LoggedNetworkNumber extenderTolerance;
    private final @NotNull LoggedNetworkNumber extenderSiftAngleOne;
    private final @NotNull LoggedNetworkNumber extenderSiftAngleTwo;

    public ExtenderIOSim() {
        this.setpoint = ExtenderConstants.kExtenderStowAngle;

        this.extenderStowAngle =
                new LoggedNetworkNumber("Intake/Extender/StowAngle", ExtenderConstants.kExtenderStowAngle.in(Degrees));
        this.extenderIntakeAngle = new LoggedNetworkNumber(
                "Intake/Extender/IntakeAngle", ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        this.extenderTolerance =
                new LoggedNetworkNumber("Intake/Extender/Tolerance", ExtenderConstants.kExtenderTolerance.in(Degrees));
        this.extenderSiftAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleOne", ExtenderConstants.kExtenderSiftAngleOne.in(Degrees));
        this.extenderSiftAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleTwo", ExtenderConstants.kExtenderSiftAngleTwo.in(Degrees));
        new LoggedNetworkNumber("Intake/Extender/DownSpeed", ExtenderConstants.kDownSpeed);

        this.armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                ExtenderConstants.kGearing,
                ExtenderConstants.kMOI.in(KilogramSquareMeters),
                ExtenderConstants.kExtenderArmLength.in(Meters),
                ExtenderConstants.kExtenderStowAngle.in(Radians),
                ExtenderConstants.kExtenderIntakeAngle.in(Radians),
                false,
                ExtenderConstants.kExtenderZeroAngle.in(Radians),
                0.0,
                0.0);

        this.pidController = new TunablePIDController(
                "Intake/ExtenderPID",
                () -> this.getPosition().in(Degrees),
                percent -> this.appliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0),
                ExtenderConstants.kExtenderConstraints);

        this.pidController.addPreset("default", ExtenderConstants.PIDF.normalPID);

        this.armMech = new LoggedMechanism2d(5, 5);
        LoggedMechanismRoot2d armMechRoot = this.armMech.getRoot("IntakeSimulation", 3, 3);
        this.armLigament = new LoggedMechanismLigament2d("arm", 2, this.extenderStowAngle.get(), 2.0, new Color8Bit(Color.kBlue));
        this.setpointArmLigament =
                new LoggedMechanismLigament2d("setpoint", 2, this.setpoint.in(Degrees), 1.0, new Color8Bit(Color.kRed));
        armMechRoot.append(this.armLigament);
        armMechRoot.append(this.setpointArmLigament);
    }

    private @NotNull Angle getPosition() {
        return Radians.of(this.armSim.getAngleRads());
    }

    private boolean isAtAngle(@NotNull Angle current, Angle target) {
        return Math.abs(current.minus(target).in(Degrees)) < this.extenderTolerance.get();
    }

    public boolean isAtAngle(Angle angle) {
        return this.isAtAngle(this.getPosition(), angle);
    }

    public void setPosition(@NotNull Angle position) {
        this.setpoint = position;
        Logger.recordOutput("Intake/Extender/SetpointDegrees", this.setpoint);
        this.setPidEnabled(true);
        this.pidController.setSetpoint(position.in(Degrees));
    }

    @Override
    public void extend() {
        this.setPosition(Degrees.of(this.extenderStowAngle.get()));
    }

    @Override
    public void retract() {
        this.setPosition(Degrees.of(this.extenderIntakeAngle.get()));
    }

    @Override
    public void goToSiftAngleOne() {
        this.setPosition(Degrees.of(this.extenderSiftAngleOne.get()));
    }

    @Override
    public void goToSiftAngleTwo() {
        this.setPosition(Degrees.of(this.extenderSiftAngleTwo.get()));
    }

    @Override
    public void toggle() {
        if (this.isRetracted().getAsBoolean()) {
            this.extend();
        } else {
            this.retract();
        }
    }

    @Override
    public void stop() {
        this.pidEnabled = false;
        this.appliedVolts = 0.0;
        this.armSim.setInputVoltage(0.0);
    }

    @Override
    public void setPidEnabled(boolean enabled) {
        this.pidEnabled = enabled;
    }

    @Override
    public void setMotorPercentage(double percent) {
        this.setPidEnabled(false);
        this.appliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
    }

    @Override
    public @NotNull BooleanSupplier isExtended() {
        return () -> this.isAtAngle(Degrees.of(this.extenderIntakeAngle.get()));
    }

    @Override
    public @NotNull BooleanSupplier isRetracted() {
        return () -> this.isAtAngle(Degrees.of(this.extenderStowAngle.get()));
    }

    @Override
    public @NotNull BooleanSupplier atTarget() {
        return () -> this.isAtAngle(this.setpoint);
    }

    @Override
    public edu.wpi.first.units.measure.@NotNull Current getCurrent() {
        return Amps.of(this.armSim.getCurrentDrawAmps());
    }

    @Override
    public void updateInputs(@NotNull ExtenderIOInputs inputs) {

        this.armSim.setInputVoltage(this.appliedVolts);
        this.armSim.update(TimedRobot.kDefaultPeriod);

        Angle position = this.getPosition();
        inputs.position = position;
        inputs.setpoint = this.setpoint;
        inputs.isExtended = this.isAtAngle(position, Degrees.of(this.extenderIntakeAngle.get()));
        inputs.isRetracted = this.isAtAngle(position, Degrees.of(this.extenderStowAngle.get()));
        inputs.atTarget = this.isAtAngle(position, this.setpoint);
        inputs.velocity = RadiansPerSecond.of(this.armSim.getVelocityRadPerSec());
        inputs.motorVoltage = Volts.of(this.appliedVolts);
        inputs.motorCurrent = Amps.of(this.armSim.getCurrentDrawAmps());
        inputs.motorTemp = Celsius.of(25.0);

        this.armLigament.setAngle(this.getPosition());
        this.setpointArmLigament.setAngle(this.setpoint);
        Logger.recordOutput("Intake/Simulation/2D-Simulation", this.armMech);

        Logger.recordOutput(
                "Intake/Simulation/3D-Simulation-Pose",
                this.armMech.generate3dMechanism().get(0));
    }

    @Override
    public void periodic() {

        this.pidController.updateTunableGains();
        if (this.pidEnabled) {
            this.pidController.runPid();
        }
    }
}
