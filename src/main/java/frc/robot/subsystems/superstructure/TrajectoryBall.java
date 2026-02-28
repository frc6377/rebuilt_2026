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

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.CalculationMode;

/** Utility class for calculating shooter trajectories, including Shooting on the Fly (SotF). */
public class TrajectoryBall {

    /** Resulting setpoints for a shooting maneuver. */
    public record ShootingParameters(Angle hoodAngle, AngularVelocity flywheelVelocity, Rotation2d targetHeading) {}

    /** Internal record for trajectory flight stats. */
    private record TrajectoryResult(Angle launchAngle, LinearVelocity launchSpeed, double totalTime) {}

    /**
     * Calculates shooting parameters for a stationary or moving robot.
     *
     * @param mode Calculation mode (Physics vs Map)
     * @param hasHood Whether the robot has an active hood subsystem
     * @param robotPose Current field pose
     * @param robotSpeeds Current field-relative or robot-relative speeds (for SotF)
     * @param maxHeight Calculated peak height of the trajectory (ignored if no hood)
     * @param targetHeight Height of the target opening
     * @param hoodAngleOffset User-defined fine-tuning offset
     * @param rpmMultiplier User-defined fine-tuning multiplier
     * @param isSotfEnabled Whether to apply movement compensation
     * @return Calculated setpoints
     */
    public static ShootingParameters calculate(
            CalculationMode mode,
            boolean hasHood,
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            Distance maxHeight,
            Distance targetHeight,
            double hoodAngleOffset,
            double rpmMultiplier,
            boolean isSotfEnabled) {
        Translation2d hubPosition = FieldConstants.getHubPosition();
        Translation2d robotPosition = robotPose.getTranslation();
        Distance staticDistance = Meters.of(robotPosition.getDistance(hubPosition));
        Rotation2d staticAngle =
                new Rotation2d(hubPosition.getX() - robotPosition.getX(), hubPosition.getY() - robotPosition.getY());

        // Field-relative robot velocity
        Translation2d robotVelocity = getRobotVelocity(robotPose, robotSpeeds);

        // 1. Initial stationary trajectory
        TrajectoryResult stationary;
        if (mode == CalculationMode.DOU_INTERPOLATION) {
            stationary = calculateStationaryMap(staticDistance);
        } else {
            if (hasHood) {
                stationary = calculateStationary(staticDistance, maxHeight, targetHeight);
            } else {
                stationary = calculateFixedAngle(staticDistance, targetHeight, ShooterConstants.kFixedHoodAngle);
            }
        }

        if (!isSotfEnabled || robotSpeeds == null || stationary.totalTime() <= 0) {
            return finalizeParameters(
                    stationary.launchAngle, stationary.launchSpeed, staticAngle, hoodAngleOffset, rpmMultiplier);
        }

        TrajectoryResult compensated =
                calculateSotf(hubPosition, robotPosition, robotVelocity, stationary, hasHood, mode, maxHeight);

        Rotation2d targetHeading =
                calculateTargetHeading(hubPosition, robotPosition, robotVelocity, stationary.totalTime());

        return finalizeParameters(
                compensated.launchAngle, compensated.launchSpeed, targetHeading, hoodAngleOffset, rpmMultiplier);
    }

    /** Calculates trajectory based on a lookup table (map) of RPM vs Distance. Assumes a fixed hood angle. */
    private static TrajectoryResult calculateStationaryMap(Distance distance) {
        // 1. Get base RPM from interpolation map
        double rpm = ShooterConstants.distanceToAngularVelocityDouMapRPM.get(distance.in(Meters));
        AngularVelocity flywheelVelocity = RPM.of(rpm);

        // 2. Convert RPM to tangential surface speed of the flywheel
        // v = omega * r
        LinearVelocity tangentialVelocity =
                MetersPerSecond.of(flywheelVelocity.in(RadiansPerSecond) * ShooterConstants.flywheelRadius.in(Meters));

        // 3. Estimate launch speed using an efficiency factor (loss during transfer to ball)
        LinearVelocity launchSpeed =
                MetersPerSecond.of(tangentialVelocity.in(MetersPerSecond) * ShooterConstants.launchEfficiency);
        Angle angle = ShooterConstants.kFixedHoodAngle;

        // 4. Calculate Time of Flight (TOF)
        // t = distance / horizontal_velocity
        // v_x = v * cos(theta)
        double totalTime = distance.in(Meters) / (launchSpeed.in(MetersPerSecond) * Math.cos(angle.in(Radians)));

        return new TrajectoryResult(angle, launchSpeed, totalTime);
    }

    /**
     * Calculates a stationary trajectory with a specific peak height (apex). Uses constant acceleration kinematics for
     * projectile motion.
     */
    private static TrajectoryResult calculateStationary(Distance distance, Distance maxHeight, Distance targetHeight) {
        Distance startHeight = ShooterConstants.shooterHeight;
        Distance riseHeight = maxHeight.minus(startHeight);
        Distance fallHeight = maxHeight.minus(targetHeight);

        if (riseHeight.in(Meters) <= 0 || fallHeight.in(Meters) < 0) {
            return new TrajectoryResult(Degrees.of(45.0), MetersPerSecond.of(0.0), 1.0);
        }

        double gravityMps2 = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
        double riseMeters = riseHeight.in(Meters);
        double fallMeters = fallHeight.in(Meters);
        double distanceMeters = distance.in(Meters);

        // 1. Calculate time to reach apex and time to fall to target
        // d = 0.5 * g * t^2  => t = sqrt(2d / g)
        double timeToRise = Math.sqrt(2.0 * riseMeters / gravityMps2);
        double timeToFall = Math.sqrt(2.0 * fallMeters / gravityMps2);
        double totalTime = timeToRise + timeToFall;

        // 2. Calculate required velocities
        // Horizontal: v_x = distance / totalTime
        // Vertical: v_y = g * timeToRise (velocity needed to reach peak height)
        double horizontalVelocity = distanceMeters / totalTime;
        double verticalVelocity = gravityMps2 * timeToRise;

        // 3. Resultant launch speed and angle
        // v = sqrt(v_x^2 + v_y^2), theta = atan2(v_y, v_x)
        Angle launchAngle = Radians.of(Math.atan2(verticalVelocity, horizontalVelocity));
        LinearVelocity launchSpeed = MetersPerSecond.of(
                Math.sqrt(horizontalVelocity * horizontalVelocity + verticalVelocity * verticalVelocity));

        return new TrajectoryResult(launchAngle, launchSpeed, totalTime);
    }

    /**
     * Calculates required velocity for a fixed launch angle to hit a target. Uses the projectile motion equation: y =
     * x*tan(theta) - (g*x^2) / (2*v^2*cos^2(theta))
     */
    private static TrajectoryResult calculateFixedAngle(Distance distance, Distance targetHeight, Angle fixedAngle) {
        double d = distance.in(Meters);
        double theta = fixedAngle.in(Radians);
        double gravity = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
        double deltaH = targetHeight.minus(ShooterConstants.shooterHeight).in(Meters);

        // Solving for v: v = sqrt( (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - deltaH)) )
        double cosTheta = Math.cos(theta);
        double denominator = 2 * cosTheta * cosTheta * (d * Math.tan(theta) - deltaH);

        if (denominator <= 0) {
            return new TrajectoryResult(fixedAngle, MetersPerSecond.of(0.0), 1.0);
        }

        double launchSpeedMps = Math.sqrt((gravity * d * d) / denominator);
        double timeOfFlight = d / (launchSpeedMps * cosTheta);

        return new TrajectoryResult(fixedAngle, MetersPerSecond.of(launchSpeedMps), timeOfFlight);
    }

    /** Converts robot-relative speeds and pose into field-relative velocity. */
    private static Translation2d getRobotVelocity(Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        Rotation2d heading = robotPose.getRotation();
        double fieldVx =
                robotSpeeds.vxMetersPerSecond * heading.getCos() - robotSpeeds.vyMetersPerSecond * heading.getSin();
        double fieldVy =
                robotSpeeds.vxMetersPerSecond * heading.getSin() + robotSpeeds.vyMetersPerSecond * heading.getCos();
        return new Translation2d(fieldVx, fieldVy);
    }

    /**
     * Compensates for robot movement (Shooting on the Fly).
     *
     * <p>The goal is for the ball's FIELD-RELATIVE velocity to match what is needed for a stationary shot. We achieve
     * this by adjusting the ROBOT-RELATIVE launch.
     *
     * <p>v_ball_field = v_ball_robot + v_robot_field Therefore: v_ball_robot = v_ball_field - v_robot_field
     */
    private static TrajectoryResult calculateSotf(
            Translation2d hubPosition,
            Translation2d robotPosition,
            Translation2d robotVelocity,
            TrajectoryResult stationary,
            boolean hasHood,
            CalculationMode mode,
            Distance maxHeight) {

        double t = stationary.totalTime();
        double fieldVx = robotVelocity.getX();
        double fieldVy = robotVelocity.getY();

        // 1. Calculate required field-relative velocity to reach hub in time t
        double desiredVfx = (hubPosition.getX() - robotPosition.getX()) / t;
        double desiredVfy = (hubPosition.getY() - robotPosition.getY()) / t;

        // 2. Calculate required robot-relative launch velocity (vector subtraction)
        double launchVxField = desiredVfx - fieldVx;
        double launchVyField = desiredVfy - fieldVy;

        double horizontalLaunchSpeed = Math.sqrt(launchVxField * launchVxField + launchVyField * launchVyField);

        // 3. Vertical velocity
        double verticalVelocity;
        if (hasHood && mode != CalculationMode.DOU_INTERPOLATION) {
            // physics mode: maintain peak height (v_y doesn't change with horizontal motion)
            double gravityMps2 = ShooterConstants.gravity.in(MetersPerSecondPerSecond);
            double riseMeters = maxHeight.minus(ShooterConstants.shooterHeight).in(Meters);
            verticalVelocity = gravityMps2 * Math.sqrt(Math.max(0, 2.0 * riseMeters / gravityMps2));
        } else {
            // fixed angle mode: v_z = horizontal_v * tan(theta)
            // theta is the fixed launch angle from the stationary shot
            verticalVelocity = horizontalLaunchSpeed * Math.tan(stationary.launchAngle.in(Radians));
        }

        // 4. Resultant launch speed and angle
        Angle launchAngle = Radians.of(Math.atan2(verticalVelocity, horizontalLaunchSpeed));
        LinearVelocity totalLaunchSpeed = MetersPerSecond.of(
                Math.sqrt(horizontalLaunchSpeed * horizontalLaunchSpeed + verticalVelocity * verticalVelocity));

        return new TrajectoryResult(launchAngle, totalLaunchSpeed, t);
    }

    /** Calculates the field-relative heading the robot should face to lead the shot. */
    private static Rotation2d calculateTargetHeading(
            Translation2d hubPosition, Translation2d robotPosition, Translation2d robotVelocity, double totalTime) {

        double fieldVx = robotVelocity.getX();
        double fieldVy = robotVelocity.getY();

        // Vector subtraction to find the direction the ball needs to be launched relative to the robot
        double desiredVfx = (hubPosition.getX() - robotPosition.getX()) / totalTime;
        double desiredVfy = (hubPosition.getY() - robotPosition.getY()) / totalTime;

        double launchVxField = desiredVfx - fieldVx;
        double launchVyField = desiredVfy - fieldVy;

        return new Rotation2d(launchVxField, launchVyField);
    }

    /** Applies offsets, clamps, and converts linear speed back to flywheel RPM. */
    private static ShootingParameters finalizeParameters(
            Angle launchAngle, LinearVelocity launchSpeed, Rotation2d heading, double hoodOffset, double rpmMult) {
        // 1. Final hood angle with manual offset and hardware limits
        double hoodDegrees = launchAngle.in(Degrees) + hoodOffset;
        hoodDegrees = Math.max(
                ShooterConstants.minHoodAngle.in(Degrees),
                Math.min(ShooterConstants.maxHoodAngle.in(Degrees), hoodDegrees));

        // 2. Linear speed back to RPM
        // omega = (v / efficiency) / r
        double flywheelRadiusMeters = ShooterConstants.flywheelRadius.in(Meters);
        double angularVelRadPerSec =
                (launchSpeed.in(MetersPerSecond) / ShooterConstants.launchEfficiency) / flywheelRadiusMeters;
        double rpm = RadiansPerSecond.of(angularVelRadPerSec).in(RPM) * rpmMult;

        rpm = Math.max(
                ShooterConstants.minShootingFlywheelVelocity.in(RPM),
                Math.min(ShooterConstants.maxShootingFlywheelVelocity.in(RPM), rpm));
        return new ShootingParameters(Degrees.of(hoodDegrees), RPM.of(rpm), heading);
    }
}
