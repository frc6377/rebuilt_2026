package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

/** Rainbow pattern cycling through predefined colors. */
public class RainbowPattern {

    private static final PatternNode[] pattern = {
        new PatternNode(RGB.RED, 10),
        new PatternNode(RGB.ORANGE, 10),
        new PatternNode(RGB.YELLOW, 10),
        new PatternNode(RGB.GREEN, 10),
        new PatternNode(RGB.HOWDY_BLUE, 10),
        new PatternNode(RGB.PURPLE, 10)
    };

    private static final int patternLength;

    static {
        int length = 0;
        for (PatternNode p : pattern) {
            length += p.repeat();
        }
        patternLength = length;
    }

    public static PatternNode[] getPattern() {
        return pattern;
    }

    public static int getPatternLength() {
        return patternLength;
    }
}
