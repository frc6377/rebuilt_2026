package frc.robot.subsystems.drive.constants;

public class NerfDriveConstants {
    public static final DriveConstants DATA = new DriveConstants(
            BaseDriveConstants.DATA.maxLinearSpeed().times(0.1),
            BaseDriveConstants.DATA.maxAngularSpeedRadPerSec() * 0.1,
            BaseDriveConstants.DATA.odometryFrequency());
}
