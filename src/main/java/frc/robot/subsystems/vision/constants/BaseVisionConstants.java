package frc.robot.subsystems.vision.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class BaseVisionConstants {
    public static final VisionConstants DATA;

    static {
        // AprilTag layout
        AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        // Camera names, must match names configured on coprocessor
        String camera0Name = "shooter";
        String camera1Name = "camera_1";

        // Camera 0 position (inches) and rotation (degrees: roll, pitch, yaw)
        // These are the single source of truth — config.json is auto-generated from them.
        double CAMERA0_X_INCHES = 0.887;
        double CAMERA0_Y_INCHES = 0.034;
        double CAMERA0_Z_INCHES = 17.878;
        double CAMERA0_ROLL_DEG = 0;
        double CAMERA0_PITCH_DEG = 30;
        double CAMERA0_YAW_DEG = 0;
        Transform3d robotToCamera0 = new Transform3d(
                Inches.of(0.887),
                Inches.of(0.034),
                Inches.of(17.878),
                new Rotation3d(Degrees.of(0), Degrees.of(30), Degrees.of(0)));
        Transform3d robotToCamera1 = new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));
        Transform3d robotToQuest =
                new Transform3d(0.0, 0.0, 0.0, new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(90)));

        DATA = new VisionConstants(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                "shooter",
                "camera_1",
                robotToCamera0,
                robotToCamera1,
                robotToQuest,
                0.3, // maxAmbiguity
                0.75, // maxZError
                0.02, // linearStdDevBaseline
                0.06, // angularStdDevBaseline
                VecBuilder.fill(0.02, 0.02, 0.035),
                new double[] {1.0, 1.0},
                0.5,
                Double.POSITIVE_INFINITY,
                2);
    }
}
