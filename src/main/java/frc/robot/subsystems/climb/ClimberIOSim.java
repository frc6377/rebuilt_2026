package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ClimberIOSim implements ClimberIO {
    // Sim
    private static LoggedMechanism2d climbMech;
    private ElevatorSim climbSim;

    private LoggedMechanismLigament2d climbLig;
    private LoggedMechanismRoot2d mechRoot;

    private double appliedVolts = 0.0;
    private Distance targetHeight = ClimbConstants.kStartHeight;
    private boolean closedLoopControl = false;

    private double integralAccumulator = 0.0;
    private double previousError = 0.0;
    private static final double DT = 0.02;

    public ClimberIOSim() {
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
        appliedVolts = percent * 12.0;
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
    public void periodic() {
        if (closedLoopControl) {
            double currentPosition = climbSim.getPositionMeters();
            double error = targetHeight.in(Meters) - currentPosition;
            
            // Proportional
            double kP = ClimbConstants.PIDF.kP;
            double proportional = kP * error;
            
            // Integral (with anti-windup)
            double kI = ClimbConstants.PIDF.kI;
            integralAccumulator += error * DT;
            // Anti-windup: clamp integral to prevent excessive buildup
            double maxIntegral = 2.0; // Max voltage contribution from integral
            integralAccumulator = Math.max(-maxIntegral / kI, Math.min(maxIntegral / kI, integralAccumulator));
            double integral = kI * integralAccumulator;
            
            // Derivative
            double kD = ClimbConstants.PIDF.kD;
            double derivative = kD * (error - previousError) / DT;
            
            // Calculate total output
            appliedVolts = proportional + integral + derivative;
            
            // Clamp to battery voltage
            appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
            
            // Store previous error for next iteration
            previousError = error;
            
            // Log PID components for debugging
            Logger.recordOutput("Climb/Simulation/PID/kP", kP);
            Logger.recordOutput("Climb/Simulation/PID/kI", kI);
            Logger.recordOutput("Climb/Simulation/PID/kD", kD);
            Logger.recordOutput("Climb/Simulation/PID/Error", error);       

        }

        climbSim.setInputVoltage(appliedVolts);
        climbSim.update(0.02);

        climbLig.setLength(Meters.of(climbSim.getPositionMeters()));

        Logger.recordOutput("Climb/Simulation/Height (Meters)", climbSim.getPositionMeters());
        Logger.recordOutput("Climb/Simulation/Velocity (M/s)", climbSim.getVelocityMetersPerSecond());
        Logger.recordOutput("Climb/Simulation/Current (Amps)", climbSim.getCurrentDrawAmps());
        Logger.recordOutput("Climb/Simulation/Applied Volts", appliedVolts);
        Logger.recordOutput("Climb/Simulation/Target Height", targetHeight.in(Meters));
        Logger.recordOutput("Climb/Simulation/2D Mech", climbMech);
    }

    @Override
    public void resetToAbsolute() {}
}
