package frc.robot.subsystems.signaling;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for the Signaling (LED) subsystem. */
public interface SignalingIO {

    @AutoLog
    class SignalingIOInputs {
        public int currentR = 0;
        public int currentG = 0;
        public int currentB = 0;
        public boolean connected = false;
    }

    /** Update the set of loggable inputs. */
    default void updateInputs(SignalingIOInputs inputs) {}

    /** Set a solid color on a section of the LED strip. */
    default void setSolidColor(int startIdx, int count, int r, int g, int b) {}

    /** Set LED brightness (0.0 to 1.0). */
    default void setBrightness(double brightness) {}

    /** Turn off all LEDs. */
    default void stop() {}
}
