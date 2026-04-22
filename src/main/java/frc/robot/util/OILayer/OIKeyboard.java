package frc.robot.util.OILayer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jetbrains.annotations.NotNull;

import java.util.function.DoubleSupplier;

public class OIKeyboard implements OI {
    private static final XboxController controller = new XboxController(0);

    // *** Keyboard 0 Mappings ***
    // Axes
    public static final DoubleSupplier AxisAD = () -> controller.getRawAxis(0);
    public static final DoubleSupplier AxisWS = () -> controller.getRawAxis(1);
    public static final DoubleSupplier AxisRE = () -> controller.getRawAxis(2);

    // Buttons
    public static final Trigger Z = new JoystickButton(controller, 1);
    public static final Trigger X = new JoystickButton(controller, 2);
    public static final Trigger C = new JoystickButton(controller, 3);
    public static final Trigger V = new JoystickButton(controller, 4);

    // POV (Meant for Numpad)
    public static final Trigger Num8 = new POVButton(controller, 0);
    public static final Trigger Num9 = new POVButton(controller, 45);
    public static final Trigger Num6 = new POVButton(controller, 90);
    public static final Trigger Num3 = new POVButton(controller, 135);
    public static final Trigger Num2 = new POVButton(controller, 180);
    public static final Trigger Num1 = new POVButton(controller, 225);
    public static final Trigger Num4 = new POVButton(controller, 270);
    public static final Trigger Num7 = new POVButton(controller, 315);

    // *** Keyboard 1 Mappings ***
    // Axies
    public static final DoubleSupplier AxisJL = () -> controller.getRawAxis(0);
    public static final DoubleSupplier AxisIK = () -> controller.getRawAxis(1);

    // Buttons
    public static final Trigger M = new JoystickButton(controller, 1);
    public static final Trigger Comma = new JoystickButton(controller, 2);
    public static final Trigger Period = new JoystickButton(controller, 3);
    public static final Trigger Slash = new JoystickButton(controller, 4);

    // *** Keyboard 2 Mappings ***
    // Axies
    public static final DoubleSupplier LeftRightArrows = () -> controller.getRawAxis(0);
    public static final DoubleSupplier UpDownArrows = () -> controller.getRawAxis(1);

    // Buttons
    public static final Trigger Insert = new JoystickButton(controller, 1);
    public static final Trigger Home = new JoystickButton(controller, 2);
    public static final Trigger PgUp = new JoystickButton(controller, 3);
    public static final Trigger Delete = new JoystickButton(controller, 4);
    public static final Trigger End = new JoystickButton(controller, 5);
    public static final Trigger PgDown = new JoystickButton(controller, 6);

    @Override
    public @NotNull DoubleSupplier driveTranslationX() {
        return AxisAD;
    }

    @Override
    public @NotNull DoubleSupplier driveTranslationY() {
        return AxisWS;
    }

    @Override
    public @NotNull DoubleSupplier driveRotation() {
        return LeftRightArrows;
    }

    @Override
    public @NotNull Trigger zeroDrivebase() {
        return Z;
    }

    @Override
    public @NotNull Trigger spinUpShooter() {
        return C;
    }

    @Override
    public @NotNull Trigger fireShooter() {
        return V;
    }

    @Override
    public @NotNull Trigger stopSuperstructure() {
        return X;
    }

    @Override
    public @NotNull Trigger driveLock0() {
        return Insert;
    }

    @Override
    public @NotNull Trigger intake() {
        return M;
    }

    @Override
    public @NotNull Trigger outtake() {
        return Comma;
    }

    @Override
    public @NotNull Trigger intakeMiddle() {
        return Home;
    }

    @Override
    public @NotNull Trigger xDrive() {
        return Home;
    }

    @Override
    public @NotNull Trigger toggleIntake() {
        return Home;
    }

    @Override
    public @NotNull Trigger testButton() {
        return End;
    }

    @Override
    public @NotNull Trigger manualHold() {
        return Slash;
    }
}
