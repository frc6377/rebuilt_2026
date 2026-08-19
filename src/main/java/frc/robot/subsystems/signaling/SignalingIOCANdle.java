package frc.robot.subsystems.signaling;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

/** Real hardware implementation of SignalingIO using a CTRE CANdle. */
public class SignalingIOCANdle implements SignalingIO {

    private final CANdle candle;

    // Reusable control request — created once, mutated per call
    private final SolidColor solidColorControl;

    // Track last-set color for inputs
    private int lastR = 0;
    private int lastG = 0;
    private int lastB = 0;
    // Track onboard 8 LED color
    private int onboardR = 0;
    private int onboardG = 0;
    private int onboardB = 0;

    public SignalingIOCANdle() {
        candle = new CANdle(SignalingConstants.candleId);

        // Configure brightness
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED = new LEDConfigs().withBrightnessScalar(SignalingConstants.ledBrightness);
        tryUntilOk(5, () -> candle.getConfigurator().apply(config, 0.25));

        // Initialize reusable control request
        solidColorControl = new SolidColor(0, 0);
    }

    @Override
    public void updateInputs(SignalingIOInputs inputs) {
        inputs.currentR = lastR;
        inputs.currentG = lastG;
        inputs.currentB = lastB;
        inputs.connected = candle.isConnected();
        inputs.onboardR = onboardR;
        inputs.onboardG = onboardG;
        inputs.onboardB = onboardB;
    }

    @Override
    public void setSolidColor(int startIdx, int count, int r, int g, int b) {
        lastR = r;
        lastG = g;
        lastB = b;
        candle.setControl(solidColorControl
                .withLEDStartIndex(startIdx)
                .withLEDEndIndex(startIdx + count)
                .withColor(new RGBWColor(r, g, b)));
    }

    @Override
    public void setOnboardColor(int r, int g, int b) {
        onboardR = r;
        onboardG = g;
        onboardB = b;
        // Onboard LEDs are indices 0-7
        candle.setControl(
                solidColorControl.withLEDStartIndex(0).withLEDEndIndex(8).withColor(new RGBWColor(r, g, b)));
    }

    @Override
    public void setBrightness(double brightness) {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED = new LEDConfigs().withBrightnessScalar(brightness);
        tryUntilOk(5, () -> candle.getConfigurator().apply(config, 0.25));
    }

    @Override
    public void stop() {
        setSolidColor(0, SignalingConstants.ledStripStart + SignalingConstants.numLEDs, 0, 0, 0);
    }
}