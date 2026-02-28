package frc.robot.subsystems.shooter.left;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/** Constants specific to the left shooter. */
public class LeftShooterConstants {
    public static final String name = "LeftShooter";

    // CAN IDs
    public static final int flywheelLeaderId = Constants.CANIDs.MotorIDs.kShooterFlywheelLeftMotorCANID;
    public static final int flywheelFollowerId = Constants.CANIDs.MotorIDs.kShooterFlywheelLeftFollowerCANID;
    public static final int spinMotorId = Constants.CANIDs.MotorIDs.kShooterSpinMotorLeftCANID;

    // CAN bus name
    public static final String canBusName = "rio";

    // Feature flags
    public static final boolean enabled = Constants.EnabledSubsystems.kShooterLeft;
    public static final boolean followerEnabled = true;
    public static final boolean spinMotorEnabled = false;

    // Motor inversion (left side is counter-clockwise positive)
    public static final InvertedValue flywheelInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue spinInverted = InvertedValue.CounterClockwise_Positive;

    // Ramp rates
    public static final Time flywheelOpenLoopRamp = Seconds.of(0.05); // seconds from 0 to full throttle
    public static final Time spinOpenLoopRamp = Seconds.of(0.05); // seconds from 0 to full throttle
    public static final Time flywheelClosedLoopRamp = Seconds.of(0.05); // seconds from 0 to full throttle
    public static final Time spinClosedLoopRamp = Seconds.of(0.05); // seconds from 0 to full throttle

    // Flywheel PID
    public static final double flywheelKP = 0.17263;
    public static final double flywheelKI = 0.0;
    public static final double flywheelKD = 0.0;
    public static final double flywheelKV = 0.11583;
    public static final double flywheelKS = 0.096109;
    public static final double flywheelKA = 0.010325;

    // Flywheel current limits
    public static final Current flywheelCurrentLimitStator = Amps.of(70);
    public static final Current flywheelCurrentLimitSupply = Amps.of(50);
    public static final boolean flywheelCurrentLimitStatorEnable = true;
    public static final boolean flywheelCurrentLimitSupplyEnable = true;

    // Spin motor PID
    public static final double spinKP = 0.3;
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
    public static final double defaultSpinRatio = 0;
}
