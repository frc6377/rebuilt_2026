package frc.robot.util.OILayer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class OIXbox implements OI {
    private static final double triggerThreshold = 0.5;

    private static final XboxController controller = new XboxController(0);

    // Face Buttons
    public static final Trigger a = new JoystickButton(controller, XboxController.Button.kA.value);
    public static final Trigger b = new JoystickButton(controller, XboxController.Button.kB.value);
    public static final Trigger x = new JoystickButton(controller, XboxController.Button.kX.value);
    public static final Trigger y = new JoystickButton(controller, XboxController.Button.kY.value);

    // Bumpers and Triggers
    public static final Trigger leftBumper = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    public static final Trigger rightBumper = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
    public static final DoubleSupplier leftTrigger =
            () -> controller.getRawAxis(XboxController.Axis.kLeftTrigger.value);
    public static final DoubleSupplier rightTrigger =
            () -> controller.getRawAxis(XboxController.Axis.kRightTrigger.value);
    public static final Trigger leftTriggerAsButton =
            new Trigger(() -> triggerThreshold < controller.getRawAxis(XboxController.Axis.kLeftTrigger.value));
    public static final Trigger rightTriggerAsButton =
            new Trigger(() -> triggerThreshold < controller.getRawAxis(XboxController.Axis.kRightTrigger.value));

    // DPad
    public static final Trigger dPadUp = new POVButton(controller, 0);
    public static final Trigger dPadDown = new POVButton(controller, 180);
    public static final Trigger dPadRight = new POVButton(controller, 90);
    public static final Trigger dPadLeft = new POVButton(controller, 270);

    // Joysticks
    public static final Trigger leftStickButton =
            new JoystickButton(controller, XboxController.Button.kLeftStick.value);
    public static final Trigger rightStickButton =
            new JoystickButton(controller, XboxController.Button.kRightStick.value);
    public static final DoubleSupplier leftX = () -> controller.getRawAxis(XboxController.Axis.kLeftX.value);
    public static final DoubleSupplier leftY = () -> controller.getRawAxis(XboxController.Axis.kLeftY.value);
    public static final DoubleSupplier rightX = () -> controller.getRawAxis(XboxController.Axis.kRightX.value);
    public static final DoubleSupplier rightY = () -> controller.getRawAxis(XboxController.Axis.kRightY.value);

    // Top Buttons
    public static final Trigger start = new JoystickButton(controller, XboxController.Button.kStart.value);
    public static final Trigger back = new JoystickButton(controller, XboxController.Button.kBack.value);

    @Override
    public DoubleSupplier driveTranslationX() {
        return leftX;
    }

    @Override
    public DoubleSupplier driveTranslationY() {
        return leftY;
    }

    @Override
    public DoubleSupplier driveRotation() {
        return rightX;
    }

    @Override
    public Trigger zeroDrivebase() {
        return start;
    }

    @Override
    public Trigger spinUpShooter() {
        return leftTriggerAsButton;
    }

    @Override
    public Trigger fireShooter() {
        return rightBumper;
    }
}
