package frc.robot.util.OILayer;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface OI {
    Trigger noButton = new Trigger(() -> false);
    DoubleSupplier noAxis = () -> 0.0;

    ControlCurve driveTranslationCurve = new ControlCurve(1, 4, 0.1, true);
    ControlCurve driveRotationCurve = new ControlCurve(1, 2, 0.1, true);
    ControlCurve driveTranslationCurveIntakeRunning = new ControlCurve(0.8, 4, 0.1, true);

    default DoubleSupplier driveTranslationX() {
        return noAxis;
    }

    default DoubleSupplier driveTranslationY() {
        return noAxis;
    }

    default DoubleSupplier driveRotation() {
        return noAxis;
    }

    default DoubleSupplier driveTranslationXIntakeRunning() {
        return noAxis;
    }

    default DoubleSupplier driveTranslationYIntakeRunning() {
        return noAxis;
    }

    default Trigger zeroDrivebase() {
        return noButton;
    }

    default Trigger stopIntake() {
        return noButton;
    }

    default Trigger shootDriver() {
        return noButton;
    }

    default void setRumble(double driverRumble, double operatorRumble) {
        return;
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

    default Trigger unjamShooter() {
        return noButton;
    }

    /* Run the rollers on the intake while held
     *
     * Subsystem: Intake - spin the rollers forward when held, stop spinning when released
     */
    default Trigger intake() {
        return noButton;
    }

    /* Run the rollers in reverse while held */
    /* Subsystem: Intake - spin the rollers backwards when held, stop spinning when released */
    default Trigger outtake() {
        return noButton;
    }

    /* When pressed, the intake will extend, and when pressed again, it will retract
    /* Be careful when retracting in case the hopper is full of balls, that it should
    /* be able to avoid damaging the robot if things are in the way.
    /* Subsystem: Intake
    */
    default Trigger toggleIntake() {
        return noButton;
    }

    default Trigger stopShooterDriver() {
        return noButton;
    }

    default Trigger xDrive() {
        return noButton;
    }

    default Trigger intakeManualExtend() {
        return noButton;
    }

    default Trigger testButton() {
        return noButton;
    }

    default Trigger intakeMiddle() {
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

    default Trigger autoSpeedMode() {
        return noButton;
    }

    default Trigger hubShootSpeed() {
        return noButton;
    }

    default Trigger towerShootSpeed() {
        return noButton;
    }

    default Trigger cornerShootSpeed() {
        return noButton;
    }

    /** Stop all superstructure mechanisms. */
    default Trigger stopSuperstructure() {
        return noButton;
    }

    /** While held, lock the drive rotation to 0 degrees. */
    default Trigger driveLock0() {
        return noButton;
    }

    default Trigger manualHold() {
        return noButton;
    }
}
