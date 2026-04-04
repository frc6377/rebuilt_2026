package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

/** Lava/fire pattern — hot whites and oranges fading into deep reds on a black background. */
public class LavaPattern {

    private static final PatternNode[] pattern = {
        new PatternNode(RGB.BLACK, 4),
        new PatternNode(new RGB(60, 0, 0), 3),
        new PatternNode(new RGB(140, 20, 0), 2),
        new PatternNode(RGB.RED, 3),
        new PatternNode(new RGB(255, 80, 0), 2),
        new PatternNode(RGB.ORANGE, 2),
        new PatternNode(new RGB(255, 200, 50), 1),
        new PatternNode(RGB.ORANGE, 2),
        new PatternNode(new RGB(255, 80, 0), 2),
        new PatternNode(RGB.RED, 3),
        new PatternNode(new RGB(140, 20, 0), 2),
        new PatternNode(new RGB(60, 0, 0), 2),
        new PatternNode(RGB.BLACK, 6),
        new PatternNode(new RGB(80, 5, 0), 2),
        new PatternNode(new RGB(200, 40, 0), 3),
        new PatternNode(new RGB(255, 130, 0), 2),
        new PatternNode(new RGB(200, 40, 0), 2),
        new PatternNode(new RGB(80, 5, 0), 2),
        new PatternNode(RGB.BLACK, 5)
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