package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.RollerConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RollerIOSim implements RollerIO {

    private final @NotNull TalonFX rollerMotor;
    private final TalonFXSimState intakeMotorSim;
    private final @NotNull IntakeSimulation intakeSim;

    private final @NotNull LoggedNetworkNumber kRollerIntakePercent;
    private final @NotNull LoggedNetworkNumber kRollerOuttakePercent;

    public RollerIOSim(@NotNull AbstractDriveTrainSimulation driveSim) {
        var config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(RollerConstants.MotorConfig.kInverted)
                        .withNeutralMode(RollerConstants.MotorConfig.kNeutralMode))
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(RollerConstants.MotorConfig.kRampPeriod))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(RollerConstants.MotorConfig.kStatorCurrentLimit));

        this.rollerMotor = new TalonFX(Constants.CANIDs.MotorIDs.kRollerLeaderMotorID);
        this.rollerMotor.getConfigurator().apply(config);
        this.intakeMotorSim = this.rollerMotor.getSimState();
        this.intakeMotorSim.setMotorType(MotorType.KrakenX60);
        this.intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveSim,
                IntakeConstants.kIntakeWidth,
                IntakeConstants.kIntakeExtension,
                IntakeConstants.kIntakeSide,
                IntakeConstants.kIntakeCapacity);

        this.kRollerIntakePercent = new LoggedNetworkNumber("Intake/Roller/IntakePercent", RollerConstants.kIntakePercent);
        this.kRollerOuttakePercent =
                new LoggedNetworkNumber("Intake/Roller/OuttakePercent", RollerConstants.kOuttakePercent);
    }

    public void setRollerSpeed(double speed) {
        this.rollerMotor.set(speed);
    }

    @Override
    public void start() {
        this.setRollerSpeed(this.kRollerIntakePercent.get());
        this.intakeSim.startIntake();
    }

    @Override
    public void stop() {
        this.rollerMotor.stopMotor();
        this.intakeSim.stopIntake();
    }

    @Override
    public void outtake() {
        this.setRollerSpeed(this.kRollerOuttakePercent.get());
        this.intakeSim.removeObtainedGamePieces(SimulatedArena.getInstance());
    }

    @Override
    public int getIntakedFuel() {
        return this.intakeSim.getGamePiecesAmount();
    }

    @Override
    public void setMotorPercentage(double percent) {
        this.rollerMotor.set(percent);
    }

    @Override
    public void setMode(NeutralModeValue mode) {
        this.rollerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(mode));
    }

    @Override
    public void updateInputs(RollerIO.@NotNull RollerIOInputs inputs) {
        inputs.leaderSpeedPercentile = this.intakeMotorSim.getMotorVoltage() / RobotController.getBatteryVoltage();
        inputs.leaderAppliedVolts = this.intakeMotorSim.getMotorVoltageMeasure();
        inputs.leaderVelocity = this.rollerMotor.getVelocity().getValue();
        inputs.leaderMotorTemp = Celsius.of(25.0);
    }
}
