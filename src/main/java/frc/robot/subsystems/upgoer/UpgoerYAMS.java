package frc.robot.subsystems.upgoer;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class UpgoerYAMS implements UpgoerIO {
    private final SmartMotorController motor;
    private final SmartMotorControllerConfig smcConfig;

    public UpgoerYAMS(SubsystemBase subsystem, int canId, String name, double invertedScale) {
        // Step 1: Create SmartMotorControllerConfig
        this.smcConfig = new SmartMotorControllerConfig(subsystem)
                .withClosedLoopController(UpgoerConstants.kP, UpgoerConstants.kI, UpgoerConstants.kD)
                .withFeedforward(new SimpleMotorFeedforward(UpgoerConstants.kS, UpgoerConstants.kV, UpgoerConstants.kA))
                .withSupplyCurrentLimit(UpgoerConstants.supplyCurrentLimit)
                .withStatorCurrentLimit(UpgoerConstants.statorCurrentLimit)
                .withInverted(invertedScale < 0)
                .withTelemetry(name, TelemetryVerbosity.HIGH);

        // Step 2: Create SmartMotorController (TalonFXWrapper)
        TalonFX talonFX = new TalonFX(canId, "rio");
        this.motor = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);
    }

    public void setSubsystem(SubsystemBase subsystem) {
        smcConfig.withSubsystem(subsystem);
    }

    @Override
    public void updateInputs(UpgoerIOInputs inputs) {
        inputs.velocity = motor.getMechanismVelocity();
        inputs.appliedVoltage = motor.getVoltage();
        inputs.current = motor.getStatorCurrent();
        inputs.temp = motor.getTemperature();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setVelocity(velocity);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
