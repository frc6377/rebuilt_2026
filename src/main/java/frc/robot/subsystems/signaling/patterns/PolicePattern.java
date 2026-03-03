package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

/** Police-style alternating red/blue flash pattern with gaps. */
public class PolicePattern {

    private static final PatternNode[] pattern = {
        new PatternNode(RGB.RED, 5),
        new PatternNode(RGB.BLACK, 5),
        new PatternNode(RGB.RED, 5),
        new PatternNode(RGB.BLACK, 10),
        new PatternNode(RGB.BLUE, 5),
        new PatternNode(RGB.BLACK, 5),
        new PatternNode(RGB.BLUE, 5),
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
