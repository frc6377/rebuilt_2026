package frc.robot.subsystems.intake.extender;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.ExtenderConstants;
import frc.robot.util.TunableTalonFX;

public class ExtenderIOReal implements ExtenderIO {

    private final TunableTalonFX extenderMotor;
    private final CANcoder extenderEncoder;
    private final CANcoderConfiguration extenderEncoderConfig;
    private final TalonFXConfiguration extenderMotorConfig;
    private final Slot0Configs extenderPID;
    private Angle setpoint;

    public ExtenderIOReal() {
        @SuppressWarnings("unused")
        Angle setpoint = Degrees.of(0.0);

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
        extenderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        extenderMotor.apply(extenderMotorConfig);
    }

    @Override
    public void setPosition(Angle position) {
        setpoint = position;
        extenderMotor.setControl(new PositionVoltage(position));
    }

    @Override
    public void zero() {
        extenderEncoder.setPosition(0.0);
    }

    @Override
    public void extend() {
        setPosition(ExtenderConstants.kExtenderIntakeAngle);
    }

    @Override
    public void retract() {
        setPosition(ExtenderConstants.kExtenderStowAngle);
    }

    @Override
    public void updateInputs(ExtenderIOInputs inputs) {
        inputs.isExtended =
                Math.abs((extenderEncoder.getPosition().getValue().minus(ExtenderConstants.kExtenderIntakeAngle))
                                .in(Degrees))
                        < ExtenderConstants.kExtenderTolerance.in(Degrees);
        inputs.isRetracted =
                Math.abs((extenderEncoder.getPosition().getValue().minus(ExtenderConstants.kExtenderStowAngle))
                                .in(Degrees))
                        < ExtenderConstants.kExtenderTolerance.in(Degrees);
        inputs.position = extenderEncoder.getAbsolutePosition().getValue();
        inputs.setpoint = setpoint;
        inputs.motorVoltage = Volts.of(extenderMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void periodic() {
        extenderMotor.updateTunableGains();
    }
}
