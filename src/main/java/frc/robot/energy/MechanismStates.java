package frc.robot.energy;

public final class MechanismStates {
    private MechanismStates() {}

    public enum Drive {
        DISABLED,
        IDLE,
        ACTIVE
    }

    public enum Roller {
        OFF,
        IDLE,
        ACTIVE
    }

    public enum Extender {
        IDLE,
        MOVING
    }

    public enum Indexer {
        OFF,
        ACTIVE
    }

    public enum Shooter {
        IDLE,
        SPINNING
    }

    public enum Upgoer {
        OFF,
        RUNNING
    }
}
