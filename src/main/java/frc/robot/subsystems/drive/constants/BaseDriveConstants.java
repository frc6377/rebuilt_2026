package frc.robot.subsystems.drive.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

public class BaseDriveConstants {
    public static final DriveConstants DATA = new DriveConstants(
            TunerConstants.kSpeedAt12Volts,
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / Drive.DRIVE_BASE_RADIUS,
            125.0 // odometryFrequency
            );
}
