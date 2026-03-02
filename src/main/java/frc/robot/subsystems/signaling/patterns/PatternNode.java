package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

/** A node in a LED pattern, representing a color repeated over a number of LEDs. */
public class PatternNode {
    public final RGB color;
    public final int repeat;

    public PatternNode(RGB color, int repeat) {
        this.color = color;
        this.repeat = repeat;
    }
}
