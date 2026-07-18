package frc.robot.util;

import frc.robot.Constants;
import frc.robot.subsystems.drive.constants.BaseDriveConstants;
import frc.robot.subsystems.drive.constants.DriveConstants;
import frc.robot.subsystems.drive.constants.NerfDriveConstants;
import frc.robot.subsystems.indexer.constants.BaseIndexerConstants;
import frc.robot.subsystems.indexer.constants.IndexerConstants;
import frc.robot.subsystems.indexer.constants.NerfIndexerConstants;
import frc.robot.subsystems.intake.BaseIntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.NerfIntakeConstants;
import frc.robot.subsystems.shooter.BaseShooterConstants;
import frc.robot.subsystems.shooter.NerfShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.upgoer.BaseUpgoerConstants;
import frc.robot.subsystems.upgoer.NerfUpgoerConstants;
import frc.robot.subsystems.upgoer.UpgoerConstants;
import frc.robot.subsystems.vision.constants.BaseVisionConstants;
import frc.robot.subsystems.vision.constants.NerfVisionConstants;
import frc.robot.subsystems.vision.constants.VisionConstants;

public record NerfModeController(Constants.NerfMode nerfMode) {
    public IndexerConstants getIndexerConstants() {
        return switch (nerfMode) {
            case FIELD, TESTING -> BaseIndexerConstants.DATA;
            case NERFED -> NerfIndexerConstants.DATA;
        };
    }

    public ShooterConstants getShooterConstants() {
        return switch (nerfMode) {
            case FIELD, TESTING -> BaseShooterConstants.DATA;
            case NERFED -> NerfShooterConstants.DATA;
        };
    }

    public UpgoerConstants getUpgoerConstants() {
        return switch (nerfMode) {
            case FIELD, TESTING -> BaseUpgoerConstants.DATA;
            case NERFED -> NerfUpgoerConstants.DATA;
        };
    }

    public IntakeConstants getIntakeConstants() {
        return switch (nerfMode) {
            case FIELD, TESTING -> BaseIntakeConstants.DATA;
            case NERFED -> NerfIntakeConstants.DATA;
        };
    }

    public VisionConstants getVisionConstants() {
        return switch (nerfMode) {
            case FIELD, TESTING -> BaseVisionConstants.DATA;
            case NERFED -> NerfVisionConstants.DATA;
        };
    }

    public DriveConstants getDriveConstants() {
        return switch (nerfMode) {
            case FIELD, TESTING -> BaseDriveConstants.DATA;
            case NERFED -> NerfDriveConstants.DATA;
        };
    }

    public double getShooterSpeed() {
        return switch (nerfMode) {
            case FIELD, TESTING -> 1.0;
            case NERFED -> 0.6;
        };
    }
}
