package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

/** A node in a LED pattern, representing a color repeated over a number of LEDs. */
public record PatternNode(RGB color, int repeat) {}