// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Commands for auto-aiming and shooting at the hub. */
public class ShootingCommands {
    // Trajectory target heights (tunable via NetworkTables)
    // Ball peaks at 8ft, lands at hub opening at 6ft
    private static final LoggedNetworkNumber maxHeightFeet =
            new LoggedNetworkNumber("Shooting/MaxHeightFeet", ShooterConstants.defaultMaxHeightFeet);
    private static final LoggedNetworkNumber targetHeightFeet =
            new LoggedNetworkNumber("Shooting/TargetHeightFeet", ShooterConstants.defaultTargetHeightFeet);

    // Fine-tuning offsets
    private static final LoggedNetworkNumber hoodAngleOffset =
            new LoggedNetworkNumber("Shooting/HoodAngleOffset", ShooterConstants.defaultHoodAngleOffset);
    private static final LoggedNetworkNumber rpmMultiplier =
            new LoggedNetworkNumber("Shooting/RPMMultiplier", ShooterConstants.defaultRpmMultiplier);

    private ShootingCommands() {}

    /** Record to hold calculated shooting parameters with type-safe units. */
    public record ShootingParameters(Angle hoodAngle, AngularVelocity flywheelVelocity) {}

    /**
     * Calculate the distance from the robot to the hub.
     *
     * @param robotPose Current robot pose
     * @return Distance to hub
     */
    public static Distance getDistanceToHub(Pose2d robotPose) {
        Translation2d hubPosition = FieldConstants.getHubPosition();
        return Meters.of(robotPose.getTranslation().getDistance(hubPosition));
    }

    /**
     * Calculate the angle from the robot to the hub.
     *
     * @param robotPose Current robot pose
     * @return Rotation2d pointing towards the hub
     */
    public static Rotation2d getAngleToHub(Pose2d robotPose) {
        Translation2d hubPosition = FieldConstants.getHubPosition();
        Translation2d toHub = hubPosition.minus(robotPose.getTranslation());
        return new Rotation2d(toHub.getX(), toHub.getY());
    }

    /**
     * Calculate shooting parameters for a trajectory that: 1. Starts at shooter height 2. Peaks at maxHeight (default
     * 8ft) 3. Lands at targetHeight (default 6ft / hub opening)
     *
     * <p>When hood is disabled, uses fixed angle and calculates velocity to compensate.
     *
     * <p>The math: - Given: start height h0, max height hMax, target height hTarget, horizontal distance d - The ball
     * rises (hMax - h0), then falls (hMax - hTarget) to reach the hub - Time to rise: t1 = sqrt(2 * (hMax - h0) / g) -
     * Time to fall: t2 = sqrt(2 * (hMax - hTarget) / g) - Total time: t = t1 + t2 - Horizontal velocity: vx = d / t -
     * Vertical velocity at launch: vy = g * t1 = sqrt(2 * g * (hMax - h0)) - Launch angle: theta = atan(vy / vx) -
     * Launch speed: v = sqrt(vx^2 + vy^2)
     *
     * @param distance Horizontal distance to hub
     * @return ShootingParameters with hood angle and flywheel velocity
     */
    public static ShootingParameters calculateShootingParameters(Distance distance) {
        // If hood is disabled, use fixed angle and calculate velocity for that angle
        if (!ShooterConstants.hoodEnabled) {
            return calculateParametersForFixedAngle(distance, ShooterConstants.fixedHoodAngle);
        }

        // Get tunable heights as Distance
        Distance maxHeight = Feet.of(maxHeightFeet.get());
        Distance targetHeight = Feet.of(targetHeightFeet.get());
        Distance startHeight = ShooterConstants.shooterHeight;

        // Calculate rise and fall distances
        Distance riseHeight = maxHeight.minus(startHeight); // How high the ball rises
        Distance fallHeight = maxHeight.minus(targetHeight); // How far it falls to reach hub

        // Ensure valid trajectory (max height must be above both start and target)
        if (riseHeight.in(Meters) <= 0 || fallHeight.in(Meters) < 0) {
            return new ShootingParameters(Degrees.of(45.0), ShooterConstants.minShootingFlywheelVelocity);
        }

        // Time calculations (using raw doubles for intermediate math)
        double gravityMps2 = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
        double riseMeters = riseHeight.in(Meters);
        double fallMeters = fallHeight.in(Meters);
        double distanceMeters = distance.in(Meters);

        double timeToRise = Math.sqrt(2.0 * riseMeters / gravityMps2); // Time from launch to peak
        double timeToFall = Math.sqrt(2.0 * fallMeters / gravityMps2); // Time from peak to hub
        double totalTime = timeToRise + timeToFall;

        // Velocity calculations
        LinearVelocity horizontalVelocity = MetersPerSecond.of(distanceMeters / totalTime);
        LinearVelocity verticalVelocity = MetersPerSecond.of(gravityMps2 * timeToRise);

        // Launch parameters
        double vx = horizontalVelocity.in(MetersPerSecond);
        double vy = verticalVelocity.in(MetersPerSecond);
        Angle launchAngle = Radians.of(Math.atan2(vy, vx));
        LinearVelocity launchSpeed = MetersPerSecond.of(Math.sqrt(vx * vx + vy * vy));

        // Apply hood angle offset
        Angle hoodAngle = Degrees.of(launchAngle.in(Degrees) + hoodAngleOffset.get());

        // Clamp hood angle to limits
        double hoodDegrees = hoodAngle.in(Degrees);
        hoodDegrees = Math.max(
                ShooterConstants.minHoodAngle.in(Degrees),
                Math.min(ShooterConstants.maxHoodAngle.in(Degrees), hoodDegrees));
        hoodAngle = Degrees.of(hoodDegrees);

        // Convert launch speed to flywheel angular velocity
        // v = omega * r, so omega = v / r
        double flywheelRadiusMeters = ShooterConstants.flywheelRadius.in(Meters);
        double launchSpeedMps = launchSpeed.in(MetersPerSecond);
        double angularVelocityRadPerSec = (launchSpeedMps / ShooterConstants.launchEfficiency) / flywheelRadiusMeters;
        AngularVelocity flywheelVelocity = RadiansPerSecond.of(angularVelocityRadPerSec * rpmMultiplier.get());

        // Clamp flywheel velocity to limits
        double rpm = flywheelVelocity.in(RPM);
        rpm = Math.max(
                ShooterConstants.minShootingFlywheelVelocity.in(RPM),
                Math.min(ShooterConstants.maxShootingFlywheelVelocity.in(RPM), rpm));
        flywheelVelocity = RPM.of(rpm);

        return new ShootingParameters(hoodAngle, flywheelVelocity);
    }

    /**
     * Calculate shooting parameters for a fixed hood angle. Uses projectile motion to determine required velocity for
     * given distance and fixed angle.
     *
     * <p>For projectile motion with fixed angle theta: - Range R = v^2 * sin(2*theta) / g (for flat ground) - Adjusted
     * for height difference: v = sqrt(g * d / sin(2*theta)) approximately
     *
     * @param distance Horizontal distance to target
     * @param fixedAngle The fixed hood angle
     * @return ShootingParameters with the fixed angle and calculated velocity
     */
    private static ShootingParameters calculateParametersForFixedAngle(Distance distance, Angle fixedAngle) {
        double distanceMeters = distance.in(Meters);
        double angleRadians = fixedAngle.in(Radians);
        double gravityMps2 = ShooterConstants.gravity.in(MetersPerSecondPerSecond);

        // For projectile motion: R = v^2 * sin(2*theta) / g
        // Solving for v: v = sqrt(R * g / sin(2*theta))
        double sin2Theta = Math.sin(2.0 * angleRadians);

        // Avoid division by zero
        if (Math.abs(sin2Theta) < 0.001) {
            return new ShootingParameters(fixedAngle, ShooterConstants.maxShootingFlywheelVelocity);
        }

        // Calculate required launch speed
        double launchSpeedMps = Math.sqrt(distanceMeters * gravityMps2 / sin2Theta);

        // Apply multiplier for tuning
        launchSpeedMps *= rpmMultiplier.get();

        // Convert to flywheel angular velocity
        double flywheelRadiusMeters = ShooterConstants.flywheelRadius.in(Meters);
        double angularVelocityRadPerSec = (launchSpeedMps / ShooterConstants.launchEfficiency) / flywheelRadiusMeters;
        AngularVelocity flywheelVelocity = RadiansPerSecond.of(angularVelocityRadPerSec);

        // Clamp flywheel velocity to limits
        double rpm = flywheelVelocity.in(RPM);
        rpm = Math.max(
                ShooterConstants.minShootingFlywheelVelocity.in(RPM),
                Math.min(ShooterConstants.maxShootingFlywheelVelocity.in(RPM), rpm));
        flywheelVelocity = RPM.of(rpm);

        return new ShootingParameters(fixedAngle, flywheelVelocity);
    }

    /** @deprecated Use calculateShootingParameters(Distance) instead */
    @Deprecated
    public static Angle calculateOptimalHoodAngle(Distance distance) {
        return calculateShootingParameters(distance).hoodAngle();
    }

    /** @deprecated Use calculateShootingParameters(Distance) instead */
    @Deprecated
    public static AngularVelocity calculateOptimalFlywheelVelocity(Distance distance) {
        return calculateShootingParameters(distance).flywheelVelocity();
    }

    /**
     * Command that continuously updates hood angle and flywheel speed based on distance to hub.
     *
     * <p>Uses trajectory calculation to hit max height of 8ft then drop into hub at 6ft.
     *
     * @param drive Drive subsystem for pose
     * @param hood Hood subsystem
     * @param shooter Shooter subsystem
     * @return Command that auto-aims the shooter
     */
    public static Command autoAimShooter(Supplier<Pose2d> poseSupplier, Hood hood, Shooter shooter) {
        return Commands.run(
                        () -> {
                            Pose2d robotPose = poseSupplier.get();
                            Distance distance = getDistanceToHub(robotPose);
                            ShootingParameters params = calculateShootingParameters(distance);
                            Angle hoodAngle = params.hoodAngle();
                            AngularVelocity flywheelVelocity = params.flywheelVelocity();

                            // Log values
                            Logger.recordOutput("Shooting/DistanceToHub", distance.in(Meters));
                            Logger.recordOutput("Shooting/CalculatedHoodAngle", hoodAngle.in(Degrees));
                            Logger.recordOutput("Shooting/CalculatedRPM", flywheelVelocity.in(RPM));

                            // Set subsystem targets (only set hood if enabled)
                            if (ShooterConstants.hoodEnabled) {
                                hood.setAngle(hoodAngle);
                            }
                            shooter.setFlywheelVelocity(flywheelVelocity);
                        },
                        hood,
                        shooter)
                .withName("AutoAimShooter");
    }

    /**
     * Command that aims the robot at the hub while driving.
     *
     * @param drive Drive subsystem
     * @param xSupplier X velocity supplier (forward/back)
     * @param ySupplier Y velocity supplier (left/right)
     * @return Command that aims at hub while allowing translation
     */
    public static Command aimAtHubWhileDriving(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> getAngleToHub(drive.getPose()))
                .withName("AimAtHub");
    }

    /**
     * Full auto-aim command: aims robot at hub AND sets hood/flywheel automatically.
     *
     * @param drive Drive subsystem
     * @param hood Hood subsystem
     * @param shooter Shooter subsystem
     * @param xSupplier X velocity supplier
     * @param ySupplier Y velocity supplier
     * @return Combined command for full auto-aim
     */
    public static Command fullAutoAim(
            Drive drive, Hood hood, Shooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return aimAtHubWhileDriving(drive, xSupplier, ySupplier)
                .alongWith(autoAimShooter(() -> drive.getPose(), hood, shooter))
                .withName("FullAutoAim");
    }
}
