package frc.robot.subsystems.signaling;

import java.util.function.IntSupplier;

/**
 * Immutable RGB color representation for LED control.
 */
public record RGB(int red, int green, int blue, int white) {
    private static final int MAX_RGB_VALUE = 255;

    // ========== Predefined Colors ==========
    public static final RGB BLACK = new RGB(0, 0, 0);
    public static final RGB WHITE = new RGB(MAX_RGB_VALUE, MAX_RGB_VALUE, MAX_RGB_VALUE);

    public static final RGB RED = new RGB(MAX_RGB_VALUE, 0, 0);
    public static final RGB ORANGE = new RGB(255, 130, 0);
    public static final RGB YELLOW = new RGB(MAX_RGB_VALUE, MAX_RGB_VALUE, 0);
    public static final RGB GREEN = new RGB(0, MAX_RGB_VALUE, 0);
    public static final RGB BLUE = new RGB(0, 0, MAX_RGB_VALUE);
    public static final RGB HOWDY_BLUE = new RGB(0, 204, 204);
    public static final RGB PURPLE = new RGB(127, 0, 255);
    public static final RGB PINK = new RGB(174, 47, 132);

    public static final RGB HOWDY_BROWN = new RGB(229, 218, 91);
    public static final RGB HOWDY_BROWN_TINT = new RGB(186, 138, 58);
    public static final RGB HOWDY_BROWN_SHADE = new RGB(78, 52, 18);

    public static final RGB FIRE_FLY_GREEN = new RGB(254, 254, 0);

    public static final RGB[] RAINBOW = new RGB[]{RED, ORANGE, YELLOW, GREEN, HOWDY_BLUE, PURPLE};

    public RGB(final int red, final int green, final int blue) {
        this(red, green, blue, 0);
    }

    /**
     * Generate a random RGB color.
     */
    public static RGB randomColor() {
        final IntSupplier rndValue = () -> (int) Math.round(Math.random() * (MAX_RGB_VALUE + 1));
        return new RGB(rndValue.getAsInt(), rndValue.getAsInt(), rndValue.getAsInt());
    }

    /**
     * Convert to hex string (e.g., "ff00ff").
     */
    public String toHex() {
        return String.format("%02x%02x%02x", red, green, blue);
    }

    @Override
    public String toString() {
        return "RGB(" + red + ", " + green + ", " + blue + ")";
    }
}
