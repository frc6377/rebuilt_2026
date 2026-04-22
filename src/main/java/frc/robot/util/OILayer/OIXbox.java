package frc.robot.util.OILayer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jetbrains.annotations.NotNull;

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
    public @NotNull DoubleSupplier driveTranslationX() {
        return leftX;
    }

    @Override
    public @NotNull DoubleSupplier driveTranslationY() {
        return leftY;
    }

    @Override
    public @NotNull DoubleSupplier driveRotation() {
        return () -> driveRotationCurve.calculate(rightX.getAsDouble());
    }

    @Override
    public @NotNull DoubleSupplier driveTranslationXIntakeRunning() {
        return () -> driveTranslationCurveIntakeRunning.calculate(leftX.getAsDouble());
    }

    @Override
    public @NotNull DoubleSupplier driveTranslationYIntakeRunning() {
        return () -> driveTranslationCurveIntakeRunning.calculate(leftY.getAsDouble());
    }

    @Override
    public void setRumble(double driverRumble, double operatorRumble) {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, driverRumble);
        operatorController.setRumble(GenericHID.RumbleType.kBothRumble, operatorRumble);
    }

    @Override
    public @NotNull Trigger zeroDrivebase() {
        return start;
    }

    @Override
    public @NotNull Trigger intakeMiddle() {
        return y;
    }

    @Override
    public @NotNull Trigger shootDriver() {
        return a;
    }

    @Override
    public @NotNull Trigger stopShooterDriver() {
        return x;
    }

    @Override
    public @NotNull Trigger intakeManualExtend() {
        return b;
    }

    @Override
    public @NotNull Trigger spinUpShooter() {
        return operatorRightBumper;
    }

    @Override
    public @NotNull Trigger fireShooter() {
        return operatorRightTriggerAsButton;
    }

    @Override
    public @NotNull Trigger unjamShooter() {
        return operatorLeftTriggerAsButton;
    }

    @Override
    public @NotNull Trigger stopSuperstructure() {
        return operatorLeftBumper;
    }

    @Override
    public @NotNull Trigger intake() {
        return rightTriggerAsButton;
    }

    @Override
    public @NotNull Trigger outtake() {
        return leftTriggerAsButton;
    }

    @Override
    public @NotNull Trigger xDrive() {
        return opX;
    }

    @Override
    public @NotNull Trigger testButton() {
        return opY;
    }

    @Override
    public @NotNull Trigger toggleIntake() {
        return rightBumper;
    }

    @Override
    public @NotNull Trigger lockOnTarget() {
        return back;
    }

    @Override
    public @NotNull Trigger declimb() {
        return dPadDown;
    }

    @Override
    public @NotNull Trigger climb_l1() {
        return dPadUp;
    }

    @Override
    public @NotNull Trigger climb_l2() {
        return dPadRight;
    }

    @Override
    public @NotNull Trigger climb_l3() {
        return dPadLeft;
    }

    @Override
    public @NotNull Trigger autoSpeedMode() {
        return operatorDPadDown;
    }

    @Override
    public @NotNull Trigger hubShootSpeed() {
        return operatorDPadLeft;
    }

    @Override
    public @NotNull Trigger towerShootSpeed() {
        return operatorDPadUp;
    }

    @Override
    public @NotNull Trigger cornerShootSpeed() {
        return operatorDPadRight;
    }

    @Override
    public @NotNull Trigger manualHold() {
        return opA;
    }
}
