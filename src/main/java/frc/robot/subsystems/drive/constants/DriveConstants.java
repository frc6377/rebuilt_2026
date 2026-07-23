package frc.robot.subsystems.drive.constants;

import edu.wpi.first.units.measure.LinearVelocity;

public record DriveConstants(
        LinearVelocity maxLinearSpeed, double maxAngularSpeedRadPerSec, double odometryFrequency) {}
