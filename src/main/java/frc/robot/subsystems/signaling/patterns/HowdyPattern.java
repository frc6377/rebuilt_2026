package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

/** Howdy pattern cycling through team colors (brown and white). */
public class HowdyPattern {

    private static final PatternNode[] pattern = {
        new PatternNode(RGB.HOWDY_BROWN, 5),
        new PatternNode(RGB.WHITE, 3),
        new PatternNode(RGB.HOWDY_BROWN_TINT, 4),
        new PatternNode(RGB.BLACK, 3),
        new PatternNode(RGB.WHITE, 2),
        new PatternNode(RGB.HOWDY_BROWN_SHADE, 3),
        new PatternNode(RGB.BLACK, 4),
        new PatternNode(RGB.HOWDY_BROWN, 3),
        new PatternNode(RGB.WHITE, 5),
        new PatternNode(RGB.BLACK, 4)
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

