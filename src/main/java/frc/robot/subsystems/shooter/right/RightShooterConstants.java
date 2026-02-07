package frc.robot.subsystems.shooter.right;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import frc.robot.Constants;

/** Constants specific to the right shooter. */
public class RightShooterConstants {
    public static final String name = "RightShooter";

    // CAN IDs
    public static final int flywheelLeaderId = Constants.CANIDs.kShooterFlywheelRightMotorCANID;
    public static final int flywheelFollowerId = Constants.CANIDs.kShooterFlywheelRightFollowerCANID;
    public static final int spinMotorId = Constants.CANIDs.kShooterSpinMotorRightCANID;

    // CAN bus name
    public static final String canBusName = "rio";

    // Feature flag
    public static final boolean enabled = false;

    // Motor inversion (right side is clockwise positive)
    public static final boolean flywheelInverted = true; // Clockwise_Positive
    public static final boolean spinInverted = true; // Clockwise_Positive

    // Flywheel PID
    public static final double flywheelKP = 0.1;
    public static final double flywheelKI = 0.0;
    public static final double flywheelKD = 0.0;
    public static final double flywheelKV = 0.0;
    public static final double flywheelKS = 0.0;

    // Flywheel current limits
    public static final Current flywheelCurrentLimitStator = Amps.of(80.0);
    public static final Current flywheelCurrentLimitSupply = Amps.of(60.0);
    public static final boolean flywheelCurrentLimitStatorEnable = true;
    public static final boolean flywheelCurrentLimitSupplyEnable = true;

    // Spin motor PID
    public static final double spinKP = 0.1;
    public static final double spinKI = 0.0;
    public static final double spinKD = 0.0;
    public static final double spinKV = 0.0;
    public static final double spinKS = 0.0;

    // Spin motor current limits
    public static final Current spinCurrentLimitStator = Amps.of(70.0);
    public static final Current spinCurrentLimitSupply = Amps.of(50.0);
    public static final boolean spinCurrentLimitStatorEnable = true;
    public static final boolean spinCurrentLimitSupplyEnable = true;

    // Spin ratio (spin motor velocity as fraction of main flywheel velocity)
    public static final double defaultSpinRatio = 0.5;
}
