package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    public Indexer(){
        // Constructor implementation
    }
    public IndexerIO indexerIO;
    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    


    }
}
