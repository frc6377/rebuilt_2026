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
     * Subsystem: Shooter - set flywheel motor(s) to "shooting speed" based on distance, set to idle speed when released
     */
    default Trigger spinUpShooter() {
        return noButton;
    }


    /* Hold this button to fire shooter. Make sure the shooter is spun up before allowing this 
     * 
     * Subsystem: Shooter - turn on the feeding motor(s) to enable shooting to occur
     */
    default Trigger fireShooter() {
        return noButton;
    }

    /* Run the rollers on the intake while held 
     * 
     * Subsystem: Intake - spin the rollers forward when held, stop spinning when released
    */
    default DoubleSupplier intake() {
        return noAxis;
    }

    /* Run the rollers in reverse while held */
    /* Subsystem: Intake - spin the rollers backwards when held, stop spinning when released */
    default DoubleSupplier outtake() {
        return noAxis;
    }

    /* When pressed, the intake will extend, and when pressed again, it will retract
    /* Be careful when retracting in case the hopper is full of balls, that it should 
    /* be able to avoid damaging the robot if things are in the way.
    /* Subsystem: Intake
    */
    default Trigger toggleIntakeState() {
        return noButton;
    }


    /* While this button is held, the robot aims for the hub and prevents the driver from updating the rotation 
     * Subsystem: Drive - whenever this is held down, set rotation based on position
    */
    default Trigger lockOnTarget() {
        return noButton;
    }

    /* Press this button to get off of the rungs in auton mode (raise the elevator) */
    /* Subsystem: Climber */
    default Trigger declimb() {
        return noButton;
    }

    /* Subsystmem: Climber */
    default Trigger climb_l1() {
        return noButton;
    }

    /* Subsystem: Climber */
    default Trigger climb_l2() {
        return noButton;
    }

    /* Subsystem: Climber */
    default Trigger climb_l3() {
        return noButton;
    }
}