package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

/**
 * Comet pattern — a bright white "head" trailing into a fading tail, scrolling across the strip.
 */
public class CometPattern {

    private static final PatternNode[] pattern = {
        new PatternNode(RGB.BLACK, 8),
        new PatternNode(RGB.WHITE, 2),
        new PatternNode(new RGB(200, 200, 200), 2),
        new PatternNode(new RGB(140, 140, 140), 2),
        new PatternNode(new RGB(80, 80, 80), 2),
        new PatternNode(new RGB(30, 30, 30), 3),
        new PatternNode(RGB.BLACK, 10),
        new PatternNode(RGB.HOWDY_BLUE, 2),
        new PatternNode(new RGB(0, 170, 170), 2),
        new PatternNode(new RGB(0, 100, 100), 2),
        new PatternNode(new RGB(0, 50, 50), 3),
        new PatternNode(RGB.BLACK, 8)
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

