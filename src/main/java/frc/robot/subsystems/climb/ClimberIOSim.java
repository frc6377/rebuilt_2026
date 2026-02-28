package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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
    // Sim
    private final TunableTalonFX climbMotor1;

    private static LoggedMechanism2d climbMech;
    private ElevatorSim climbSim;

    private LoggedMechanismLigament2d climbLig;
    private LoggedMechanismRoot2d mechRoot;
    private final PIDController pidController;

    private double appliedVolts = 0;
    private Distance targetHeight = ClimbConstants.kStartHeight;
    private boolean closedLoopControl = ClimbConstants.kClosedLoopControl;

    // private double integralAccumulator = ClimbConstants.kIntegralAccumulator;
    // private double previousError = ClimbConstants.kPreviousError;
    // private static final double DT = ClimbConstants.kDT;

    public ClimberIOSim() {
        climbMotor1 = new TunableTalonFX(MotorIDs.kClimbMotor1ID, "sim", "ClimbMotor1");
        tryUntilOk(5, () -> climbMotor1.getConfigurator().apply(ClimbConstants.kClimbMotorConfigSim, 0.25));

        pidController =
                new PIDController(ClimbConstants.SimPIDF.kP, ClimbConstants.SimPIDF.kI, ClimbConstants.SimPIDF.kD);
        climbSim = new ElevatorSim(
                ClimbConstants.kClimbGearBox,
                ClimbConstants.kClimbGearRatio,
                ClimbConstants.kCarriageMass.in(Kilograms),
                ClimbConstants.kElevatorDrumRadius.in(Meters),
                ClimbConstants.kClimbMinHeight.in(Meters),
                ClimbConstants.kClimbMaxHeight.in(Meters),
                ClimbConstants.kSimulateGravity,
                ClimbConstants.kStartHeight.in(Meters));

        climbMech = new LoggedMechanism2d(Meters.of(2), Meters.of(2));
        mechRoot = climbMech.getRoot("root", 1, 0);
        climbLig = new LoggedMechanismLigament2d("Climber Mech [0]", 1, 90, 10, new Color8Bit(Color.kBlue));
        SmartDashboard.putData("Mech2Ds/Elevator Mech", climbMech);
        mechRoot.append(climbLig);
    }

    @Override
    public void goToHeight(Distance height) {
        pidController.setSetpoint(height.in(Meters));
        targetHeight = height;
        closedLoopControl = true;
    }

    @Override
    public void stop() {
        appliedVolts = 0.0;
        closedLoopControl = false;
    }

    @Override
    public void set(double percent) {
        climbMotor1.set(percent);
        closedLoopControl = false;
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double positionMeters = climbSim.getPositionMeters();
        inputs.height = Meters.of(positionMeters);

        double rotations = (positionMeters * ClimbConstants.kClimbGearRatio)
                / ClimbConstants.kElevatorDrumCircumference.in(Meters);
        inputs.motorPosition = Rotations.of(rotations);

        inputs.appliedVoltage = Volts.of(appliedVolts);
        inputs.statorCurrent = Amps.of(climbSim.getCurrentDrawAmps());
        inputs.supplyCurrent = Amps.of(climbSim.getCurrentDrawAmps());
        inputs.temperatureCelsius = 25.0;

        inputs.absoluteEncoderPosition = rotations % 1.0;

        inputs.motorConnected = true;
    }

    @Override
    public Distance getHeight() {
        return Meters.of(climbSim.getPositionMeters());
    }

    @Override
    public void periodic() {
        set(pidController.calculate(getHeight().in(Meters)));

        Logger.recordOutput("Climb/Simulation/PID/kP", pidController.getP());
        Logger.recordOutput("Climb/Simulation/PID/kI", pidController.getI());
        Logger.recordOutput("Climb/Simulation/PID/kD", pidController.getD());
        Logger.recordOutput("Climb/Simulation/PID/Error", pidController.getPositionError());

        climbSim.setInputVoltage(climbMotor1.getMotorVoltage().getValueAsDouble());
        climbSim.update(TimedRobot.kDefaultPeriod);

        climbLig.setLength(Meters.of(climbSim.getPositionMeters()));

        Logger.recordOutput("Climb/Simulation/Height (Meters)", getHeight());
        Logger.recordOutput("Climb/Simulation/Velocity (M/s)", climbSim.getVelocityMetersPerSecond());
        Logger.recordOutput("Climb/Simulation/Current (Amps)", climbSim.getCurrentDrawAmps());
        Logger.recordOutput(
                "Climb/Simulation/Applied Volts", climbMotor1.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Climb/Simulation/Target Height", targetHeight.in(Meters));
        Logger.recordOutput("Climb/Simulation/2D Mech", climbMech);
    }

    @Override
    public void resetToAbsolute() {}
}
