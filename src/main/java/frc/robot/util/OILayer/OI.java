package frc.robot.util.OILayer;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface OI {
    public final Trigger noButton = new Trigger(() -> false);
    public final DoubleSupplier noAxis = () -> 0.0;

    public final ControlCurve driveTranslationCurve = new ControlCurve(1, 3, 0.2, true);
    public final ControlCurve driveRotationCurve = new ControlCurve(1, 3, 0.2, true);

    default DoubleSupplier driveTranslationX() {
        return noAxis;
    }

    default DoubleSupplier driveTranslationY() {
        return noAxis;
    }

    default DoubleSupplier driveRotation() {
        return noAxis;
    }

    default Trigger zeroDrivebase() {
        return noButton;
    }

    /* Puts the shooter into a mode where it is able to shoot (e.g. spins up a flywheel that was currently idle) 
     * Rumbles joystick or turns on lights on the robot when it gets up to speed to be able to shoot
     * When button is release, return to an idle speed
    */
    default Trigger spinUpShooter() {
        return noButton;
    }


    /* Hold this button to fire shooter. Make sure the shooter is spun up before allowing this  */
    default Trigger fireShooter() {
        return noButton;
    }

    /* Run the rollers on the intake while held */
    default DoubleSupplier intake() {
        return noAxis;
    }

    /* Run the rollers in reverse while held */
    default DoubleSupplier outtake() {
        return noAxis;
    }

    /* Push the button to make the intake extend from the robot*/
    default Trigger extendIntake() {
        return noButton;
    }

    /* Push the button to make the intake retract
     * We need to make sure that nothing breaks when retracting if balls are in the hopper, 
     * if this is also tied to the hopper.
     * Have a safety mechanism if it is not retracting that it goes back out.
     */
    default Trigger retractIntake() {
        return noButton;
    }


    /* While this button is held, the robot aims for the hub and prevents the driver from updating the rotation */
    default Trigger lockOnTarget() {
        return noButton;
    }

    /* Press this button to get off of the rungs in auton mode (raise the elevator) */
    default Trigger declimb() {
        return noButton;
    }

    
    default Trigger climb_l1() {
        return noButton;
    }

    default Trigger climb_l2() {
        return noButton;
    }

    default Trigger climb_l3() {
        return noButton;
    }
}