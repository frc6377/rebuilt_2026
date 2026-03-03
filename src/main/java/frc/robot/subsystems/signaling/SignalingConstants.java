package frc.robot.subsystems.signaling;

import frc.robot.Constants;

/** Shared constants for the Signaling subsystem. */
public final class SignalingConstants {

    // Feature flags
    public static final boolean enabled = Constants.EnabledSubsystems.kSignaling;

    // CAN IDs
    public static final int candleId = Constants.CANIDs.SensorIDs.kCANdleID;

    // LED configuration
    public static final int ledStripStart = 8; // First LED index after onboard CANdle LEDs
    public static final int numLEDs = 60; // Number of LEDs on the strip (excluding onboard)
    public static final int totalLEDs = ledStripStart + numLEDs; // Total LED count including onboard 8
    public static final double ledBrightness = 1.0; // 0.0 to 1.0

    // Pattern timing
    public static final double patternSpeed = 0.05; // Seconds between pattern ticks
    public static final int patternTicksPerUpdate = 1; // Pattern ticks before advancing pattern
}
