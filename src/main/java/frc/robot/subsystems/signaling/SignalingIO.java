package frc.robot.subsystems.signaling;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Signaling (LED) subsystem. */
public interface SignalingIO {

    @AutoLog
    class SignalingIOInputs {
        // Strip color (last set on main strip)
        public int currentR = 0;
        public int currentG = 0;
        public int currentB = 0;
        public boolean connected = false;
        // Onboard 8 LED colors (indices 0-7)
        public int onboardR = 0;
        public int onboardG = 0;
        public int onboardB = 0;
        // Per-LED state for sim visualization (packed 0xRRGGBB, length = totalLEDs)
        public int[] ledColors = new int[SignalingConstants.totalLEDs];
    }

    /** Update the set of loggable inputs. */
    default void updateInputs(SignalingIOInputs inputs) {}

    /** Set a solid color on a section of the LED strip. */
    default void setSolidColor(int startIdx, int count, int r, int g, int b) {}

    /** Set a solid color on the 8 onboard CANdle LEDs (indices 0-7). */
    default void setOnboardColor(int r, int g, int b) {}

    /** Set LED brightness (0.0 to 1.0). */
    default void setBrightness(double brightness) {}

    /** Turn off all LEDs. */
    default void stop() {}
}