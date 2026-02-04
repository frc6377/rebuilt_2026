package frc.robot.subsystems.indexer;
public interface IndexerIO {
    default void index() {}
    default void indexReverse() {}
    default void stop() {}
}