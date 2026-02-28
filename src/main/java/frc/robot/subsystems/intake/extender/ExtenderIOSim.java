package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
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

    private final TalonFXConfiguration extenderMotorConfig;
    private final TunableTalonFX extenderMotor;
    private final Slot0Configs extenderPID;
    private final TalonFXSimState extenderMotorSim;
    private final LoggedMechanism2d armMech;
    private final LoggedMechanismRoot2d armMechRoot;
    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d setpointArmLigament;
    private final CurrentLimitsConfigs currentConfig;
    private final SingleJointedArmSim armSim;
    private Angle setpoint;
    // Logged network numbers for tuning/monitoring extender angles (dashboard-tunable)
    private final LoggedNetworkNumber kExtenderStowAngle;
    private final LoggedNetworkNumber kExtenderIntakeAngle;
    private final LoggedNetworkNumber kExtenderMaxAngle;
    private final LoggedNetworkNumber kExtenderMinAngle;
    private final LoggedNetworkNumber kExtenderTolerance;
    private final LoggedNetworkNumber kExtenderSiftAngleOne;
    private final LoggedNetworkNumber kExtenderSiftAngleTwo;

    public ExtenderIOSim() {
        setpoint = Degrees.of(0.0);

        extenderPID = new Slot0Configs();
        extenderPID.kP = ExtenderConstants.PIDF.kP;
        extenderPID.kI = ExtenderConstants.PIDF.kI;
        extenderPID.kD = ExtenderConstants.PIDF.kD;

        extenderMotor = new TunableTalonFX(
                Constants.CANIDs.MotorIDs.kExtenderMotorID, "rio", "Intake/ExtenderPID", extenderPID);

        currentConfig = new CurrentLimitsConfigs();
        currentConfig.StatorCurrentLimitEnable = true;
        currentConfig.StatorCurrentLimit = ExtenderConstants.MotorConfig.kStatorCurrentLimitExtender.in(Amps);

        extenderMotorConfig = new TalonFXConfiguration();
        extenderMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ExtenderConstants.MotorConfig.kRampPeriod;
        extenderMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extenderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extenderMotorConfig.Feedback.SensorToMechanismRatio = ExtenderConstants.kGearing;

        extenderMotor.applyConfiguration(extenderMotorConfig);
        extenderMotor.getConfigurator().apply(currentConfig);

        // Initialize LoggedNetworkNumber instances for angle constants (only angles initialized in constructor)
        kExtenderStowAngle =
                new LoggedNetworkNumber("Intake/Extender/StowAngle", ExtenderConstants.kExtenderStowAngle.in(Degrees));
        kExtenderIntakeAngle = new LoggedNetworkNumber(
                "Intake/Extender/IntakeAngle", ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        kExtenderMaxAngle =
                new LoggedNetworkNumber("Intake/Extender/MaxAngle", ExtenderConstants.kExtenderMaxAngle.in(Degrees));
        kExtenderMinAngle =
                new LoggedNetworkNumber("Intake/Extender/MinAngle", ExtenderConstants.kExtenderMinAngle.in(Degrees));
        kExtenderTolerance =
                new LoggedNetworkNumber("Intake/Extender/Tolerance", ExtenderConstants.kExtenderTolerance.in(Degrees));
        kExtenderSiftAngleOne = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleOne", ExtenderConstants.kExtenderSiftAngleOne.in(Degrees));
        kExtenderSiftAngleTwo = new LoggedNetworkNumber(
                "Intake/Extender/SiftAngleTwo", ExtenderConstants.kExtenderSiftAngleTwo.in(Degrees));

        extenderMotorSim = extenderMotor.getSimState();

        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                ExtenderConstants.kGearing,
                ExtenderConstants.kMOI.in(KilogramMetersSquaredPerSecond),
                ExtenderConstants.kExtenderArmLength.in(Meters),
                Degrees.of(kExtenderStowAngle.get()).in(Radians),
                Degrees.of(kExtenderIntakeAngle.get()).in(Radians),
                false,
                Degrees.of(kExtenderIntakeAngle.get()).in(Radians),
                0.0,
                0.0);

        armMech = new LoggedMechanism2d(5, 5);
        armMechRoot = armMech.getRoot("IntakeSimulation", 3, 3);
        armLigament = new LoggedMechanismLigament2d("arm", 2, kExtenderIntakeAngle.get());
        setpointArmLigament = new LoggedMechanismLigament2d("setpoint", 2, setpoint.in(Degrees));
        armMechRoot.append(armLigament);
        armMechRoot.append(setpointArmLigament);
    }

    public void setPosition(Angle position) {
        this.setpoint =
                Degrees.of(Math.max(kExtenderMinAngle.get(), Math.min(kExtenderMaxAngle.get(), position.in(Degrees))));
        extenderMotor.setControl(new PositionVoltage(this.setpoint));
    }

    public Angle getPosition() {
        return extenderMotor.getPosition().getValue();
    }

    public boolean isAtAngle(Angle angle) {
        return Math.abs((getPosition().minus(angle)).in(Degrees)) < kExtenderTolerance.get();
    }

    @Override
    public void zero() {
        extenderMotor.setPosition(0.0);
    }

    @Override
    public void extend() {
        setPosition(Degrees.of(kExtenderIntakeAngle.get()));
    }

    @Override
    public void retract() {
        setPosition(Degrees.of(kExtenderStowAngle.get()));
    }

    @Override
    public BooleanSupplier isExtended() {
        return () -> isAtAngle(Degrees.of(kExtenderIntakeAngle.get()));
    }

    @Override
    public BooleanSupplier isRetracted() {
        return () -> isAtAngle(Degrees.of(kExtenderStowAngle.get()));
    }

    @Override
    public BooleanSupplier atTarget() {
        return () -> isAtAngle(setpoint);
    }

    @Override
    public void goToSiftAngleOne() {
        setPosition(Degrees.of(kExtenderSiftAngleOne.get()));
    }

    @Override
    public void goToSiftAngleTwo() {
        setPosition(Degrees.of(kExtenderSiftAngleTwo.get()));
    }

    @Override
    public void toggle() {
        if (isAtAngle(Degrees.of(kExtenderIntakeAngle.get()))) {
            retract();
        } else {
            extend();
        }
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
        inputs.isExtended = isExtended().getAsBoolean();
        inputs.isRetracted = isRetracted().getAsBoolean();
        inputs.position = getPosition();
        inputs.setpoint = setpoint;
        inputs.velocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
        inputs.motorVoltage = Volts.of(extenderMotorSim.getMotorVoltage());
        inputs.motorCurrent = Amps.of(armSim.getCurrentDrawAmps());
        inputs.motorTemp = Celsius.of(25.0);
        inputs.atTarget = atTarget().getAsBoolean();
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
