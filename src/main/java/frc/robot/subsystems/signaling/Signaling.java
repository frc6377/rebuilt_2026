package frc.robot.subsystems.signaling;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.signaling.patterns.AlliancePattern;
import frc.robot.subsystems.signaling.patterns.FireflyPattern;
import frc.robot.subsystems.signaling.patterns.PatternNode;
import frc.robot.subsystems.signaling.patterns.RainbowPattern;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Signaling subsystem controlling CANdle LED strip for robot state indication. */
public class Signaling extends SubsystemBase {

    private final SignalingIO io;
    private final SignalingIOInputsAutoLogged inputs = new SignalingIOInputsAutoLogged();

    // ========== State Machine ==========

    /** Light states representing robot status. */
    public enum LightState {
        IDLE,
        HAS_FUEL,
        READY_TO_SHOOT,
        LL_HAS_TAG,
        AUTO_ALIGNING,
        SCORING
    }

    /** Disabled-mode pattern choices. */
    private enum DisablePattern {
        RAINBOW,
        ALLIANCE,
        FIREFLY;

        public static DisablePattern getRandom() {
            DisablePattern[] allPatterns = DisablePattern.values();
            return allPatterns[(int) (Math.random() * allPatterns.length)];
        }
    }

    @AutoLogOutput(key = "Signaling/CurrentState")
    private LightState currentState = LightState.IDLE;

    private DisablePattern disablePattern = DisablePattern.FIREFLY;

    // Pattern animation state
    private int tick = 0;
    private int patternTick = 0;

    // ========== State Suppliers ==========

    private Supplier<Boolean> hasFuel = () -> false;
    private Supplier<Boolean> readyToShoot = () -> false;
    private Supplier<Boolean> limelightHasTag = () -> false;
    private Supplier<Boolean> autoAligning = () -> false;
    private Supplier<Boolean> scoring = () -> false;

    public Signaling(SignalingIO io) {
        this.io = io;
    }

    // ========== Supplier Setters ==========

    /** Set supplier for whether the robot has a fuel game piece. */
    public void setHasFuel(Supplier<Boolean> hasFuel) {
        this.hasFuel = hasFuel;
    }

    /** Set supplier for whether the shooter is ready to fire. */
    public void setReadyToShoot(Supplier<Boolean> readyToShoot) {
        this.readyToShoot = readyToShoot;
    }

    /** Set supplier for whether the limelight has an AprilTag lock. */
    public void setLimelightHasTag(Supplier<Boolean> limelightHasTag) {
        this.limelightHasTag = limelightHasTag;
    }

    /** Set supplier for whether the robot is auto-aligning. */
    public void setAutoAligning(Supplier<Boolean> autoAligning) {
        this.autoAligning = autoAligning;
    }

    /** Set supplier for whether the robot is actively scoring. */
    public void setScoring(Supplier<Boolean> scoring) {
        this.scoring = scoring;
    }

    // ========== Periodic ==========

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Signaling", inputs);

        if (DriverStation.isDisabled()) {
            updatePattern();
        } else if (SignalingConstants.enabled) {
            updateStateMachine();
        }

        // Log state
        Logger.recordOutput("Signaling/HubActive", FieldConstants.isHubActive());
        Logger.recordOutput("Signaling/HasFuel", hasFuel.get());
        Logger.recordOutput("Signaling/ReadyToShoot", readyToShoot.get());
        Logger.recordOutput("Signaling/LimelightHasTag", limelightHasTag.get());
        Logger.recordOutput("Signaling/AutoAligning", autoAligning.get());
        Logger.recordOutput("Signaling/Scoring", scoring.get());
        Logger.recordOutput("Signaling/DisablePattern", disablePattern.toString());
        Logger.recordOutput(
                "Signaling/CurrentCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
    }

    // ========== State Machine Logic ==========

    private void updateStateMachine() {
        switch (currentState) {
            case IDLE:
                // Continuously refresh so hub-active / alliance changes are reflected immediately
                setFullStrip(getIdleColor());
                if (readyToShoot.get()) {
                    setState(LightState.READY_TO_SHOOT);
                }
                break;
            case HAS_FUEL:
                if (!hasFuel.get()) {
                    setState(LightState.IDLE);
                } else if (readyToShoot.get()) {
                    setState(LightState.READY_TO_SHOOT);
                }
                break;
            case READY_TO_SHOOT:
                if (!readyToShoot.get()) {
                    setState(LightState.IDLE);
                } else if (limelightHasTag.get()) {
                    setState(LightState.LL_HAS_TAG);
                }
                if (!hasFuel.get()) {
                    setState(LightState.IDLE);
                }
                break;
            case LL_HAS_TAG:
                if (!limelightHasTag.get()) {
                    setState(LightState.READY_TO_SHOOT);
                } else if (autoAligning.get()) {
                    setState(LightState.AUTO_ALIGNING);
                }
                if (!readyToShoot.get()) {
                    setState(LightState.HAS_FUEL);
                }
                if (!hasFuel.get()) {
                    setState(LightState.IDLE);
                }
                break;
            case AUTO_ALIGNING:
                if (!autoAligning.get()) {
                    setState(LightState.LL_HAS_TAG);
                }
                if (scoring.get()) {
                    setState(LightState.SCORING);
                }
                break;
            case SCORING:
                if (!scoring.get()) {
                    setState(LightState.IDLE);
                }
                break;
            default:
                break;
        }
    }

    /** Returns the color to show in the IDLE state: alliance color if our hub is active, red if not. */
    private RGB getIdleColor() {
        if (FieldConstants.isHubActive()) {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            return getColorFromAlliance(alliance);
        }
        return RGB.HOWDY_BLUE;
    }

    private void setState(LightState newState) {
        currentState = newState;
        switch (newState) {
            case IDLE:
                setFullStrip(getIdleColor());
                break;
            case HAS_FUEL:
                setFullStrip(RGB.ORANGE);
                break;
            case READY_TO_SHOOT:
                setFullStrip(RGB.WHITE);
                break;
            case LL_HAS_TAG:
                setFullStrip(RGB.GREEN);
                break;
            case AUTO_ALIGNING:
                setFullStrip(RGB.BLUE);
                break;
            case SCORING:
                setFullStrip(RGB.PURPLE);
                break;
            default:
                break;
        }
    }

    // ========== LED Control Helpers ==========

    private void setFullStrip(RGB rgb) {
        Logger.recordOutput("Signaling/LEDColor", rgb.toHex());
        io.setSolidColor(SignalingConstants.ledStripStart, SignalingConstants.numLEDs, rgb.red(), rgb.green(), rgb.blue());
    }

    private void setSection(RGB rgb, int startIdx, int count) {
        io.setSolidColor(startIdx, count, rgb.red(), rgb.green(), rgb.blue());
    }

    private void setSectionStrip(RGB rgb, int startIdx, int count) {
        if (startIdx == 10) {
            Logger.recordOutput("Signaling/LEDColor", rgb.toHex());
        }
        setSection(rgb, startIdx + SignalingConstants.ledStripStart, count);
    }

    /** Get alliance color as RGB. */
    public static RGB getColorFromAlliance(Alliance alliance) {
        if (alliance == Alliance.Red) {
            return RGB.RED;
        }
        return RGB.BLUE;
    }

    // ========== Disabled-Mode Pattern Animation ==========

    private void updatePattern() {
        tick++;
        if (tick > SignalingConstants.patternSpeed * 50) {
            tick = 0;
            patternTick++;
        } else {
            return;
        }

        PatternNode[] pattern;
        int patternLength;

        switch (disablePattern) {
            case RAINBOW:
                pattern = RainbowPattern.getPattern();
                patternLength = RainbowPattern.getPatternLength();
                break;
            case ALLIANCE:
                pattern = AlliancePattern.getPattern();
                patternLength = AlliancePattern.getPatternLength();
                break;
            case FIREFLY:
                pattern = FireflyPattern.getPattern();
                patternLength = FireflyPattern.getPatternLength();
                break;
            default:
                pattern = AlliancePattern.getPattern();
                patternLength = AlliancePattern.getPatternLength();
                break;
        }
        applyPattern(pattern, patternLength);
    }

    private void applyPattern(PatternNode[] pattern, int patternLength) {
        int patternIndex = 0;
        patternTick %= patternLength;
        int ledIndex = -patternTick - 1;
        while (ledIndex < SignalingConstants.numLEDs) {
            patternIndex %= pattern.length;
            PatternNode node = pattern[patternIndex];
            setSectionStrip(node.color(), ledIndex + 9, node.repeat());
            ledIndex += node.repeat();
            patternIndex++;
        }
    }

    /** Randomize the disabled-mode pattern. */
    public void randomizePattern() {
        disablePattern = DisablePattern.getRandom();
    }

    /** Clear all LEDs to off. */
    public void clearLEDs() {
        setFullStrip(RGB.BLACK);
    }

    // ========== Command Factory Methods ==========

    /** Set a specific light state. */
    public Command setStateCommand(LightState state) {
        return Commands.runOnce(() -> setState(state), this).withName("SetState" + state.name());
    }

    /** Set a solid color on the LED strip. */
    public Command setColorCommand(RGB rgb) {
        return Commands.runOnce(() -> setFullStrip(rgb), this).withName("SetColor");
    }

    /** Set LEDs to alliance color, reset on end. */
    public Command setToAllianceCommand() {
        return Commands.startEnd(
                        () -> {
                            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                            setFullStrip(getColorFromAlliance(alliance));
                        },
                        this::clearLEDs,
                        this)
                .withName("SetToAlliance");
    }

    /** Signal with a color (LEDs only), clears on end. */
    public Command signalCommand(RGB color) {
        return Commands.startEnd(() -> setFullStrip(color), this::clearLEDs, this)
                .withName("Signal");
    }

    /** Reset to idle state. */
    public Command idleCommand() {
        return Commands.runOnce(() -> setState(LightState.IDLE), this).withName("SignalingIdle");
    }

    /** Stop all LEDs. */
    public Command stopCommand() {
        return Commands.runOnce(this::clearLEDs, this).withName("SignalingStop");
    }
}
