package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramMetersSquaredPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunableTalonFX;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ExtenderIOSim implements ExtenderIO {

    private final TalonFXConfiguration extenderMotorConfig;
    private final TunableTalonFX extenderMotor;
    private final CANcoder extenderEncoder;
    private final CANcoderConfiguration extenderEncoderConfig;
    private final CANcoderSimState extenderEncoderSim;
    private final Slot0Configs extenderPID;
    private final TalonFXSimState extenderMotorSim;
    private final LoggedMechanism2d armMech;
    private final LoggedMechanismRoot2d armMechRoot;
    private final LoggedMechanismLigament2d armLigament;
    private final LoggedMechanismLigament2d setpointArmLigament;
    private final SingleJointedArmSim armSim;
    private Angle setpoint;

    public ExtenderIOSim() {
        setpoint = Degrees.of(0.0);

        extenderEncoder = new CANcoder(Constants.CANIDs.SensorIDs.kExtenderEncoderID);

        extenderEncoderConfig = new CANcoderConfiguration();
        extenderEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        extenderEncoder.getConfigurator().apply(extenderEncoderConfig);

        extenderPID = new Slot0Configs();
        extenderPID.kP = ExtenderConstants.PIDF.kP;
        extenderPID.kI = ExtenderConstants.PIDF.kI;
        extenderPID.kD = ExtenderConstants.PIDF.kD;

        extenderMotor =
                new TunableTalonFX(Constants.CANIDs.MotorIDs.kExtenderMotorID, "rio", "Intake/Extender", extenderPID);

        extenderMotorConfig = new TalonFXConfiguration();
        extenderMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
        extenderMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        extenderMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        extenderMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        extenderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        extenderMotor.apply(extenderMotorConfig);

        extenderMotorSim = extenderMotor.getSimState();
        extenderEncoderSim = extenderEncoder.getSimState();

        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                ExtenderConstants.kGearing,
                ExtenderConstants.kMOI.in(KilogramMetersSquaredPerSecond),
                ExtenderConstants.kExtenderArmLength.in(Meters),
                ExtenderConstants.kExtenderIntakeAngle.in(Radians),
                ExtenderConstants.kExtenderStowAngle.in(Radians),
                false,
                ExtenderConstants.kExtenderIntakeAngle.in(Radians),
                0.0);

        armMech = new LoggedMechanism2d(5, 5);
        armMechRoot = armMech.getRoot("IntakeSimulation", 3, 3);
        armLigament = new LoggedMechanismLigament2d("arm", 2, ExtenderConstants.kExtenderIntakeAngle.in(Degrees));
        setpointArmLigament = new LoggedMechanismLigament2d("setpoint", 2, setpoint.in(Degrees));
        armMechRoot.append(armLigament);
    }

    @Override
    public void setPosition(Angle position) {
        setpoint = position;
        extenderMotor.setControl(new PositionVoltage(position));
        extenderEncoderSim.setRawPosition(position);
    }

    @Override
    public void zero() {
        extenderEncoder.setPosition(0.0);
    }

    @Override
    public void extend() {
        setPosition(ExtenderConstants.kExtenderStowAngle);
    }

    @Override
    public void retract() {
        setPosition(ExtenderConstants.kExtenderIntakeAngle);
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
        inputs.isExtended = armSim.hasHitUpperLimit();
        inputs.isRetracted = armSim.hasHitLowerLimit();
        inputs.position = Radians.of(armSim.getAngleRads());
        inputs.setpoint = setpoint;
        inputs.motorVoltage = Volts.of(extenderMotorSim.getMotorVoltage());
    }

    @Override
    public void periodic() {
        armSim.setInputVoltage(extenderMotorSim.getMotorVoltage());
        armSim.update(0.02);
        extenderEncoderSim.setRawPosition(armSim.getAngleRads());

        armLigament.setAngle(extenderEncoder.getPosition().getValue());
        setpointArmLigament.setAngle(setpoint);
        Logger.recordOutput("Intake/2D-Simulation", armMech);

        extenderMotor.updateTunableGains();
    }
}
