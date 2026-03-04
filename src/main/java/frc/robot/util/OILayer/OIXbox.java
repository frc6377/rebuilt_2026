package frc.robot.util.OILayer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class OIXbox implements OI {
    private static final double triggerThreshold = 0.5;

    private static final XboxController driverController = new XboxController(0);
    private static final XboxController operatorController = new XboxController(1);

    // Face Buttons
    public static final Trigger a = new JoystickButton(driverController, XboxController.Button.kA.value);
    public static final Trigger b = new JoystickButton(driverController, XboxController.Button.kB.value);
    public static final Trigger x = new JoystickButton(driverController, XboxController.Button.kX.value);
    public static final Trigger y = new JoystickButton(driverController, XboxController.Button.kY.value);

    // Opertator face buttons
    public static final Trigger opA = new JoystickButton(operatorController, XboxController.Button.kA.value);
    public static final Trigger opB = new JoystickButton(operatorController, XboxController.Button.kB.value);
    public static final Trigger opX = new JoystickButton(operatorController, XboxController.Button.kX.value);
    public static final Trigger opY = new JoystickButton(operatorController, XboxController.Button.kY.value);

    // Bumpers and Triggers
    public static final Trigger leftBumper =
            new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    public static final Trigger rightBumper =
            new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    public static final DoubleSupplier leftTrigger =
            () -> driverController.getRawAxis(XboxController.Axis.kLeftTrigger.value);
    public static final DoubleSupplier rightTrigger =
            () -> driverController.getRawAxis(XboxController.Axis.kRightTrigger.value);
    public static final Trigger leftTriggerAsButton =
            new Trigger(() -> triggerThreshold < driverController.getRawAxis(XboxController.Axis.kLeftTrigger.value));
    public static final Trigger rightTriggerAsButton =
            new Trigger(() -> triggerThreshold < driverController.getRawAxis(XboxController.Axis.kRightTrigger.value));

    // Operator Bumpers and Triggers
    public static final Trigger operatorLeftBumper =
            new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    public static final Trigger operatorRightBumper =
            new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    public static final DoubleSupplier operatorLeftTrigger =
            () -> operatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value);
    public static final DoubleSupplier operatorRightTrigger =
            () -> operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value);
    public static final Trigger operatorLeftTriggerAsButton =
            new Trigger(() -> triggerThreshold < operatorController.getRawAxis(XboxController.Axis.kLeftTrigger.value));
    public static final Trigger operatorRightTriggerAsButton = new Trigger(
            () -> triggerThreshold < operatorController.getRawAxis(XboxController.Axis.kRightTrigger.value));

    // DPad
    public static final Trigger dPadUp = new POVButton(driverController, 0);
    public static final Trigger dPadDown = new POVButton(driverController, 180);
    public static final Trigger dPadRight = new POVButton(driverController, 90);
    public static final Trigger dPadLeft = new POVButton(driverController, 270);

    public static final Trigger operatorDPadUp = new POVButton(operatorController, 0);
    public static final Trigger operatorDPadRight = new POVButton(operatorController, 90);
    public static final Trigger operatorDPadDown = new POVButton(operatorController, 180);

    public static final Trigger operatorDPadLeft = new POVButton(operatorController, 270);

    // Joysticks
    public static final Trigger leftStickButton =
            new JoystickButton(driverController, XboxController.Button.kLeftStick.value);
    public static final Trigger rightStickButton =
            new JoystickButton(driverController, XboxController.Button.kRightStick.value);
    public static final DoubleSupplier leftX = () -> driverController.getRawAxis(XboxController.Axis.kLeftX.value);
    public static final DoubleSupplier leftY = () -> driverController.getRawAxis(XboxController.Axis.kLeftY.value);
    public static final DoubleSupplier rightX = () -> driverController.getRawAxis(XboxController.Axis.kRightX.value);
    public static final DoubleSupplier rightY = () -> driverController.getRawAxis(XboxController.Axis.kRightY.value);

    // Top Buttons
    public static final Trigger start = new JoystickButton(driverController, XboxController.Button.kStart.value);
    public static final Trigger back = new JoystickButton(driverController, XboxController.Button.kBack.value);

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
        return () -> driveRotationCurve.calculate(rightX.getAsDouble());
    }

    @Override
    public DoubleSupplier driveTranslationXIntakeRunning() {
        return () -> driveTranslationCurveIntakeRunning.calculate(leftX.getAsDouble());
    }

    @Override
    public DoubleSupplier driveTranslationYIntakeRunning() {
        return () -> driveTranslationCurveIntakeRunning.calculate(leftY.getAsDouble());
    }

    @Override
    public Trigger zeroDrivebase() {
        return start;
    }

    @Override
    public Trigger driveLock0() {
        return a;
    }

    @Override
    public Trigger intakeMiddle() {
        return y;
    }

    @Override
    public Trigger spinUpShooter() {
        return operatorRightBumper;
    }

    @Override
    public Trigger fireShooter() {
        return operatorRightTriggerAsButton;
    }

    @Override
    public Trigger unjamShooter() {
        return operatorLeftTriggerAsButton;
    }

    @Override
    public Trigger stopSuperstructure() {
        return operatorLeftBumper;
    }

    @Override
    public Trigger intake() {
        return rightTriggerAsButton;
    }

    @Override
    public Trigger outtake() {
        return leftTriggerAsButton;
    }

    @Override
    public Trigger zeroIntake() {
        return opX;
    }

    @Override
    public Trigger downIntake() {
        return opY;
    }

    @Override
    public Trigger toggleIntakeState() {
        return rightBumper;
    }

    @Override
    public Trigger lockOnTarget() {
        return back;
    }

    @Override
    public Trigger declimb() {
        return dPadDown;
    }

    @Override
    public Trigger climb_l1() {
        return dPadUp;
    }

    @Override
    public Trigger climb_l2() {
        return dPadRight;
    }

    @Override
    public Trigger climb_l3() {
        return dPadLeft;
    }

    @Override
    public Trigger shootSpeedLow() {
        return operatorDPadDown;
    }

    @Override
    public Trigger shootSpeedMidLow() {
        return operatorDPadLeft;
    }

    @Override
    public Trigger shootSpeedMidHigh() {
        return operatorDPadUp;
    }

    @Override
    public Trigger shootSpeedHigh() {
        return operatorDPadRight;
    }
}
