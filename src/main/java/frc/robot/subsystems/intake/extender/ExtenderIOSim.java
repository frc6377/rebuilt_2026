package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunableTalonFX;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ExtenderIOSim implements ExtenderIO {

    private final TunableTalonFX extenderMotor;
    private final Slot0Configs extenderPID;
    private final TalonFXSimState extenderMotorSim;
    private final LoggedMechanism2d armMech;
    private final LoggedMechanismRoot2d armMechRoot;
    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d setpointArmLigament;
    private final SingleJointedArmSim armSim;
    private Angle setpoint;
    private final LoggedNetworkNumber extenderStowAngle;
    private final LoggedNetworkNumber extenderIntakeAngle;
    private final LoggedNetworkNumber extenderTolerance;
    private final LoggedNetworkNumber extenderSiftAngleOne;
    private final LoggedNetworkNumber extenderSiftAngleTwo;
    private boolean pidEnabled = true;

    public ExtenderIOSim() {
        setpoint = Degrees.of(0.0);

        extenderPID = new Slot0Configs();
        extenderPID.kP = ExtenderConstants.PIDF.kP;
        extenderPID.kI = ExtenderConstants.PIDF.kI;
        extenderPID.kD = ExtenderConstants.PIDF.kD;

        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(ExtenderConstants.MotorConfig.kInverted)
                        .withNeutralMode(ExtenderConstants.MotorConfig.kNeutralMode))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(ExtenderConstants.MotorConfig.kRampPeriod))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(ExtenderConstants.MotorConfig.kStatorCurrentLimitExtender))
                .withSlot0(new Slot0Configs()
                        .withKP(ExtenderConstants.PIDF.kP)
                        .withKI(ExtenderConstants.PIDF.kI)
                        .withKD(ExtenderConstants.PIDF.kD)
                        .withKS(ExtenderConstants.PIDF.kS)
                        .withKV(ExtenderConstants.PIDF.kV)
                        .withKA(ExtenderConstants.PIDF.kA));

        extenderMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kExtenderMotorID, "rio", "Intake/ExtenderPID", extenderPID);
        extenderMotor.applyConfiguration(config);

        extenderStowAngle =
                new LoggedNetworkNumber("Intake/Extender/StowAngle", ExtenderConstants.kExtenderStowAngle.in(Degrees));
        extenderIntakeAngle = new LoggedNetworkNumber(
                "Intake/Extender/IntakeAngle", ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        extenderTolerance =
                new LoggedNetworkNumber("Intake/Extender/Tolerance", ExtenderConstants.kExtenderTolerance.in(Degrees));
        extenderSiftAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleOne", ExtenderConstants.kExtenderSiftAngleOne.in(Degrees));
        extenderSiftAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleTwo", ExtenderConstants.kExtenderSiftAngleTwo.in(Degrees));
        new LoggedNetworkNumber("Intake/Extender/DownSpeed", ExtenderConstants.kDownSpeed);

        extenderMotorSim = extenderMotor.getSimState();

        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                ExtenderConstants.kGearing,
                ExtenderConstants.kMOI.in(KilogramMetersSquaredPerSecond),
                ExtenderConstants.kExtenderArmLength.in(Meters),
                Degrees.of(extenderStowAngle.get()).in(Radians),
                Degrees.of(extenderIntakeAngle.get()).in(Radians),
                false,
                Degrees.of(extenderIntakeAngle.get()).in(Radians),
                0.0,
                0.0);

        armMech = new LoggedMechanism2d(5, 5);
        armMechRoot = armMech.getRoot("IntakeSimulation", 3, 3);
        armLigament = new LoggedMechanismLigament2d("arm", 2, extenderIntakeAngle.get());
        setpointArmLigament = new LoggedMechanismLigament2d("setpoint", 2, setpoint.in(Degrees));
        armMechRoot.append(armLigament);
        armMechRoot.append(setpointArmLigament);
    }

    public void setPosition(Angle position) {
        this.setpoint = position;
        Logger.recordOutput("Intake/Extender/SetpointDegrees", this.setpoint);
        setPidEnabled(true);
        extenderMotor.setControl(new PositionVoltage(this.setpoint));
    }

    public Angle getPosition() {
        return extenderMotor.getPosition().getValue();
    }

    public boolean isAtAngle(Angle angle) {
        return isAtAngle(getPosition(), angle);
    }

    private boolean isAtAngle(Angle current, Angle target) {
        return Math.abs((current.minus(target)).in(Degrees)) < extenderTolerance.get();
    }

    @Override
    public void zero() {
        extenderMotor.setPosition(0.0);
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
        extenderMotor.stopMotor();
    }

    @Override
    public void setPidEnabled(boolean enabled) {
        pidEnabled = enabled;
        if (enabled) {
            extenderMotor.setControl(new PositionVoltage(setpoint));
        } else {
            extenderMotor.stopMotor();
        }
    }

    @Override
    public void setMode(NeutralModeValue mode) {
        extenderMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
    }

    @Override
    public void setMotorPercentage(double percent) {
        setPidEnabled(false);
        extenderMotor.set(percent);
    }

    @Override
    public edu.wpi.first.units.measure.Current getCurrent() {
        return Amps.of(armSim.getCurrentDrawAmps());
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
        Angle position = getPosition();
        inputs.position = position;
        inputs.setpoint = setpoint;
        inputs.isExtended = isAtAngle(position, Degrees.of(extenderIntakeAngle.get()));
        inputs.isRetracted = isAtAngle(position, Degrees.of(extenderStowAngle.get()));
        inputs.atTarget = isAtAngle(position, setpoint);
        inputs.velocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
        inputs.motorVoltage = Volts.of(extenderMotorSim.getMotorVoltage());
        inputs.motorCurrent = Amps.of(armSim.getCurrentDrawAmps());
        inputs.motorTemp = Celsius.of(25.0);
    }

    @Override
    public void periodic() {
        armSim.setInputVoltage(extenderMotorSim.getMotorVoltage());
        armSim.update(TimedRobot.kDefaultPeriod);
        extenderMotor.setPosition(armSim.getAngleRads());

        armLigament.setAngle(getPosition());
        setpointArmLigament.setAngle(setpoint);
        Logger.recordOutput("Intake/2D-Simulation", armMech);

        extenderMotor.updateTunableGains();
    }
}
