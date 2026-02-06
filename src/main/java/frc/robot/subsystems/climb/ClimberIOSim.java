package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClimberIOSim implements ClimberIO {
    // Sim
    private static LoggedMechanism2d climbMech;
    private ElevatorSim climbSim;

    private LoggedMechanismLigament2d climbLig;
    private LoggedMechanismRoot2d mechRoot;

    // gearbox The type of and number of motors in the elevator gearbox.

    // gearing The gearing of the elevator (numbers greater than 1 represent reductions).

    // carriageMassKg The mass of the elevator carriage.

    // drumRadiusMeters The radius of the drum that the elevator spool is wrapped around.

    // minHeightMeters The min allowable height of the elevator.

    // maxHeightMeters The max allowable height of the elevator.

    // simulateGravity Whether gravity should be simulated or not.

    // startingHeightMeters The starting height of the elevator.

    // measurementStdDevs The standard deviations of the measurements. Can be omitted if no noise is desired. If present
    // must have 1 element for position.

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
        mechRoot.append(climbLig);
    }

    @Override
    public void set(double percent) {
        climbSim.setInput(percent * 12);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double positionMeters = climbSim.getPositionMeters();
        inputs.height = Meters.of(positionMeters);
    }

    @Override
    public void periodic() {
        climbSim.update(0.02);
        climbLig.setLength(Meters.of(climbSim.getPositionMeters()));
        Logger.recordOutput("Elevator/Elv/Setpoint (Inches)", 0.0);
        Logger.recordOutput("Elevator/Elv/Setpoint (Rotations)", 0.0);
        Logger.recordOutput("Elevator/Simulation/2dsim", climbMech);
    }
}
