package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

/** Firefly-style pattern with green flashes on a dark background. */
public class FireflyPattern {

    private static final PatternNode[] pattern = {
        new PatternNode(RGB.BLACK, 5),
        new PatternNode(RGB.FIRE_FLY_GREEN, 2),
        new PatternNode(RGB.BLACK, 8),
        new PatternNode(RGB.FIRE_FLY_GREEN, 1),
        new PatternNode(RGB.BLACK, 12),
        new PatternNode(RGB.FIRE_FLY_GREEN, 3),
        new PatternNode(RGB.BLACK, 6),
        new PatternNode(RGB.FIRE_FLY_GREEN, 1),
        new PatternNode(RGB.BLACK, 10)
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