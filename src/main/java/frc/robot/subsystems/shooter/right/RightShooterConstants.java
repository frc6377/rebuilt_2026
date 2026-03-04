package frc.robot.subsystems.shooter.right;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/** Constants specific to the right shooter. */
public class RightShooterConstants {
    public static final String name = "RightShooter";

    // CAN IDs
    public static final int flywheelLeaderId = Constants.CANIDs.MotorIDs.kShooterFlywheelRightMotorCANID;
    public static final int flywheelFollowerId = Constants.CANIDs.MotorIDs.kShooterFlywheelRightFollowerCANID;
    public static final int spinMotorId = Constants.CANIDs.MotorIDs.kShooterSpinMotorRightCANID;

    // CAN bus name
    public static final String canBusName = "rio";

    // Feature flags
    public static final boolean enabled = Constants.EnabledSubsystems.kShooterRight;
    public static final boolean followerEnabled = false;
    public static final boolean spinMotorEnabled = false;

    // Motor inversion (right side is clockwise positive)
    public static final InvertedValue flywheelInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue spinInverted = InvertedValue.Clockwise_Positive;

    // Ramp rates
    public static final Time flywheelOpenLoopRamp = Seconds.of(0.05); // seconds from 0 to full throttle
    public static final Time spinOpenLoopRamp = Seconds.of(0.05); // seconds from 0 to full throttle
    public static final Time flywheelClosedLoopRamp = Seconds.of(0.05); // seconds from 0 to full throttle
    public static final Time spinClosedLoopRamp = Seconds.of(0.05); // seconds from 0 to full throttle

    // Flywheel PID
    public static final double flywheelKP = 0.6;
    public static final double flywheelKI = 0.0;
    public static final double flywheelKD = 0.0;
    public static final double flywheelKV = 0.0;
    public static final double flywheelKS = 0.0;
    public static final double flywheelKA = 0.0;

    // Flywheel current limits
    public static final Current flywheelCurrentLimitStator = Amps.of(60.0);
    public static final Current flywheelCurrentLimitSupply = Amps.of(40.0);
    public static final boolean flywheelCurrentLimitStatorEnable = true;
    public static final boolean flywheelCurrentLimitSupplyEnable = true;

    // Spin motor PID
    public static final double spinKP = 0.3;
    public static final double spinKI = 0.0;
    public static final double spinKD = 0.0;
    public static final double spinKV = 0.0;
    public static final double spinKS = 0.0;

    // Spin motor current limits
    public static final Current spinCurrentLimitStator = Amps.of(60.0);
    public static final Current spinCurrentLimitSupply = Amps.of(40.0);
    public static final boolean spinCurrentLimitStatorEnable = true;
    public static final boolean spinCurrentLimitSupplyEnable = true;

    // Default ratios
    public static final double defaultSpinRatio = 0.0;
}
