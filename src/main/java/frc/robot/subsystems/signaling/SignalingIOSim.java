package frc.robot.subsystems.signaling;

/** Simulation implementation of SignalingIO. Logs colors without real hardware. */
public class SignalingIOSim implements SignalingIO {

    // Track last-set color for inputs
    private int lastR = 0;
    private int lastG = 0;
    private int lastB = 0;

    @Override
    public void updateInputs(SignalingIOInputs inputs) {
        inputs.currentR = lastR;
        inputs.currentG = lastG;
        inputs.currentB = lastB;
        inputs.connected = true; // Always connected in sim
    }

    @Override
    public void setSolidColor(int startIdx, int count, int r, int g, int b) {
        lastR = r;
        lastG = g;
        lastB = b;
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
    }
}
