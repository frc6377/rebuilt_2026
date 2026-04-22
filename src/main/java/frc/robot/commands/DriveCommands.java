// Copyright 2021-2025 FRC 6328
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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.OILayer.OI;
import org.jetbrains.annotations.NotNull;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = OI.driveTranslationCurve.calculate(Math.hypot(x, y));
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /** Field relative drive command using two joysticks (controlling linear and angular velocities). */
    public static @NotNull Command joystickDrive(
            @NotNull Drive drive, @NotNull DoubleSupplier xSupplier, @NotNull DoubleSupplier ySupplier, @NotNull DoubleSupplier omegaSupplier) {
        return joystickDrive(drive, xSupplier, ySupplier, omegaSupplier, () -> false);
    }

    /** Field relative drive command using two joysticks with X mode support. */
    public static @NotNull Command joystickDrive(
            @NotNull Drive drive,
            @NotNull DoubleSupplier xSupplier,
            @NotNull DoubleSupplier ySupplier,
            @NotNull DoubleSupplier omegaSupplier,
            java.util.function.@NotNull BooleanSupplier xModeSupplier) {
        return Commands.run(
                () -> {
                    if (xModeSupplier.getAsBoolean()
                            && 0.2 > Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble())) {
                        drive.stop();
                        drive.stopWithX();
                    } else {

                        // Get linear velocity
                        Translation2d linearVelocity =
                                getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                        // Apply rotation deadband
                        double omega = omegaSupplier.getAsDouble();

                        // Convert to field relative speeds & send command
                        ChassisSpeeds speeds = new ChassisSpeeds(
                                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                omega * drive.getMaxAngularSpeedRadPerSec());
                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                && Alliance.Red == DriverStation.getAlliance().get();
                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                speeds,
                                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
                    }
                },
                drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control. Possible use cases
     * include snapping to an angle, aiming at a vision target, or controlling absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            @NotNull Drive drive, @NotNull DoubleSupplier xSupplier, @NotNull DoubleSupplier ySupplier, @NotNull Supplier<Rotation2d> rotationSupplier) {
        return joystickDriveAtAngle(drive, xSupplier, ySupplier, rotationSupplier, () -> false);
    }

    public static Command joystickDriveAtAngle(
            @NotNull Drive drive,
            @NotNull DoubleSupplier xSupplier,
            @NotNull DoubleSupplier ySupplier,
            @NotNull Supplier<Rotation2d> rotationSupplier,
            java.util.function.@NotNull BooleanSupplier xModeSupplier) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            if (xModeSupplier.getAsBoolean()) {
                                drive.stopWithX();
                                drive.stop();
                                return;
                            }

                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // Calculate angular speed
                            double omega = angleController.calculate(
                                    drive.getRotation().getRadians(),
                                    rotationSupplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega);
                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && Alliance.Red == DriverStation.getAlliance().get();
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
    }

    /**
     * Rotates the robot to center a visible AprilTag in the camera frame while still allowing full joystick
     * translation.
     *
     * <p>The {@code txSupplier} should return the horizontal angle error to the target tag in radians (positive =
     * target is to the left of center). When no tag is visible, supply {@code Rotation2d.fromDegrees(0)} and the robot
     * will hold its current heading.
     *
     * <p>The command never finishes on its own — bind it with {@code whileTrue}.
     *
     * @param drive The drive subsystem.
     * @param xSupplier Joystick translation X (forward).
     * @param ySupplier Joystick translation Y (strafe).
     * @param txSupplier Supplier of the camera's horizontal angle to the tag (Rotation2d). Return {@code null} or
     *     {@code Rotation2d.fromDegrees(0)} when no tag seen.
     * @param tagVisible Supplier that returns {@code true} when a target tag is being tracked.
     */
    public static Command aimAtTagCommand(
            @NotNull Drive drive,
            @NotNull DoubleSupplier xSupplier,
            @NotNull DoubleSupplier ySupplier,
            @NotNull Supplier<Rotation2d> txSupplier,
            java.util.function.@NotNull BooleanSupplier tagVisible) {

        // PID on robot-relative yaw error: setpoint = current heading + tx offset
        ProfiledPIDController aimController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        aimController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(
                        () -> {
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            double omega;
                            if (tagVisible.getAsBoolean() && null != txSupplier.get()) {
                                // Target heading = current heading adjusted by the camera tx error.
                                // Negated because positive tx means the tag is left of center, so we
                                // need to rotate left (positive omega in WPILib convention) to center it.
                                double targetHeading = drive.getRotation().getRadians()
                                        - txSupplier.get().getRadians();
                                omega = aimController.calculate(
                                        drive.getRotation().getRadians(), targetHeading);
                            } else {
                                // No tag visible — hold current heading
                                omega = aimController.calculate(
                                        drive.getRotation().getRadians(),
                                        drive.getRotation().getRadians());
                            }

                            boolean isFlipped = DriverStation.getAlliance().isPresent()
                                    && Alliance.Red == DriverStation.getAlliance().get();
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    new ChassisSpeeds(
                                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                            omega),
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                        },
                        drive)
                .beforeStarting(() -> aimController.reset(drive.getRotation().getRadians()))
                .withName("AimAtTag");
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static @NotNull Command feedforwardCharacterization(@NotNull Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(
                                () -> {
                                    drive.runCharacterization(0.0);
                                },
                                drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Drive FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                        }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static @NotNull Command wheelRadiusCharacterization(@NotNull Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> {
                            limiter.reset(0.0);
                        }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(() -> {
                            state.positions = drive.getWheelRadiusCharacterizationPositions();
                            state.lastAngle = drive.getRotation();
                            state.gyroDelta = 0.0;
                        }),

                        // Update gyro delta
                        Commands.run(() -> {
                                    var rotation = drive.getRotation();
                                    state.gyroDelta += Math.abs(
                                            rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(() -> {
                                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; 4 > i; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(wheelRadius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(wheelRadius))
                                            + " inches");
                                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
