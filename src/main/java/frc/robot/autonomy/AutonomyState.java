package frc.robot.autonomy;

/** High-level states for the autonomy-first robot brain. */
public enum AutonomyState {
    DISABLED,
    DISARMED,
    SELECT_OBJECTIVE,
    PATH_TO_OBJECTIVE,
    EXECUTE_OBJECTIVE,
    RECOVER,
    FAULTED
}
