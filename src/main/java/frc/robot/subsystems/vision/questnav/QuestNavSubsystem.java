package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuestNavSubsystem extends SubsystemBase {

    private QuestNavIO io;

    public QuestNavSubsystem(QuestNavIO io) {
        this.io = io;
    }
    
     @Override
     public void periodic() {}
}
