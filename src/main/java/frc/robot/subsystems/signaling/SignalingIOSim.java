package frc.robot.subsystems.signaling;

import java.util.Arrays;

/** Simulation implementation of SignalingIO. Tracks every LED individually for visualization. */
public class SignalingIOSim implements SignalingIO {

    // Full LED buffer — index matches physical CANdle LED index (0 = first onboard LED)
    private final int[] ledBuffer = new int[SignalingConstants.totalLEDs];

    // Track last-set strip color for currentR/G/B inputs
    private int lastR = 0;
    private int lastG = 0;
    private int lastB = 0;

    // Track onboard color
    private int onboardR = 0;
    private int onboardG = 0;
    private int onboardB = 0;

    @Override
    public void updateInputs(SignalingIOInputs inputs) {
        inputs.currentR = lastR;
        inputs.currentG = lastG;
        inputs.currentB = lastB;
        inputs.connected = true; // Always connected in sim
        inputs.onboardR = onboardR;
        inputs.onboardG = onboardG;
        inputs.onboardB = onboardB;
        // Copy full LED buffer so AdvantageKit logs every LED
        inputs.ledColors = Arrays.copyOf(ledBuffer, ledBuffer.length);
    }

    @Override
    public void setSolidColor(int startIdx, int count, int r, int g, int b) {
        lastR = r;
        lastG = g;
        lastB = b;
        int packed = packRGB(r, g, b);
        int end = Math.min(startIdx + count, SignalingConstants.totalLEDs);
        for (int i = Math.max(0, startIdx); i < end; i++) {
            ledBuffer[i] = packed;
        }
    }

    @Override
    public void setOnboardColor(int r, int g, int b) {
        onboardR = r;
        onboardG = g;
        onboardB = b;
        int packed = packRGB(r, g, b);
        // Onboard LEDs are indices 0-7
        for (int i = 0; i < SignalingConstants.ledStripStart; i++) {
            ledBuffer[i] = packed;
        }
    }

    @Override
    public void setBrightness(double brightness) {
        // No-op in sim
    }

    @Override
    public void stop() {
        lastR = 0;
        lastG = 0;
        lastB = 0;
        onboardR = 0;
        onboardG = 0;
        onboardB = 0;
        Arrays.fill(ledBuffer, 0);
    }

    /** Pack r, g, b (0-255 each) into a single 0xRRGGBB int. */
    private static int packRGB(int r, int g, int b) {
        return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
    }
}
