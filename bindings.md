# Controller Bindings

## Driver Controller (Xbox)

| Key | Action | Description |
| :--- | :--- | :--- |
| Left Stick / Right Stick X | Move / Rotate | Controls the field-relative movement and rotation of the swerve drivebase. |
| A | Shoot | Initiates the shooting sequence. When the **Operator A** button is **NOT** held, the robot automatically aims at the hub, calculates the required flywheel speed based on distance, and fires once ready. If **Operator A** **IS** held, it fires using the manually selected D-Pad speed without auto-aiming. |
| X | Stop Shooter | A stop for the superstructure that instantly cuts power to the shooter flywheels and the upgoer (feeder) mechanism. |
| Y | Intake Middle | Commands the intake to move to a predefined "middle" angle, useful for specific robot states or clearing obstructions. |
| Right Bumper | Toggle Intake | Toggles the intake mechanism between its fully extended (deploy) and retracted (stow) positions. |
| Right Trigger | Intake & Index | Runs the intake rollers and the internal indexer forward to collect game pieces from the floor and move them into the robot. |
| Left Trigger | Outtake | Reverses both the intake rollers and the indexer to eject game pieces or clear a jam at the front of the robot. |
| Start | Zero Drivebase | Resets the robot's gyroscope heading and odometry. In simulation, it resets the robot to its starting pose; in a real match, it aligns the heading based on the current alliance color. |

---

## Operator Controller (Xbox)

| Key | Action | Description |
| :--- | :--- | :--- |
| A | Manual Mode Modifier | While held, this button changes the behavior of the **Right Trigger** (on both controllers). It disables the vision-based auto-aiming and distance-based speed calculation, forcing the shooter to use the RPM manually selected via the D-Pad. |
| X | Zero Intake | Resets the intake's encoder/sensor positions to ensure accurate positioning during the match. |
| Left Bumper | Stop Superstructure | Functions as a secondary emergency stop to immediately halt all shooter and upgoer/feeder movements. |
| Right Bumper | Spin Up & Auto-Fire | Spins the flywheels to the currently selected manual speed. Once the flywheels reach the target velocity, it automatically triggers the indexer and upgoer to fire any loaded game pieces. |
| Right Trigger | Shoot | Mirrors the Driver's A button functionality. It triggers the shooting sequence, which defaults to Auto-Aim/Auto-Speed unless the **Operator A** button is held. |
| Left Trigger | Unjam | Simultaneously reverses the internal upgoer mechanism and the shooter's feed system to clear game pieces that may be stuck inside the superstructure. |
| D-Pad | Manual Speeds | Used to set the manual flywheel velocity. **Up** sets 3200 RPM (Tower), **Left** sets 2600 RPM (Hub), and **Right** sets 3800 RPM (Corner). **Down** switches the manual speed logic back into an automatic calculation mode. |
