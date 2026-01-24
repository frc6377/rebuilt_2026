// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Fuel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

/** Add your docs here. */
public class fuel extends GamePieceProjectile {
    private static final double FUEL_RADIUS_METERS = 0.15; // 7 inches diameter = ~0.178m
    private static final double FUEL_MASS_KG = 0.210; // Approximately 180 grams

    public fuel(
            Translation2d robotPosition,
            Translation2d shooterPositionOnRobot,
            ChassisSpeeds chassisSpeedsRobotRelative,
            Rotation2d shooterFacing,
            Distance initialHeight,
            LinearVelocity initialSpeed,
            Angle shooterAngle) {

        super(
                new GamePieceInfo(
                        "Fuel",
                        Geometry.createCircle(FUEL_RADIUS_METERS),
                        edu.wpi.first.units.Units.Meters.of(FUEL_RADIUS_METERS * 2),
                        edu.wpi.first.units.Units.Kilograms.of(FUEL_MASS_KG),
                        0.5,
                        0.5,
                        0.8),
                robotPosition,
                shooterPositionOnRobot,
                chassisSpeedsRobotRelative,
                shooterFacing,
                initialHeight,
                initialSpeed,
                shooterAngle);
    }
}
