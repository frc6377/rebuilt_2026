package frc.robot.subsystems.indexer;

public interface IndexerIO {
    public static class IndexerIOInputs {
        // Add any sensor inputs or state variables here if needed
        public void updateInputs(IndexerIOInputs indexerInputs) {
            // Update input values from sensors or other sources here
        }
    }

    default void index() {}

    default void indexReverse() {}

    default void stop() {}

    default void setCustomSpeed(double speed) {}

    void updateInputs(IndexerIOInputs indexerInputs);
}
