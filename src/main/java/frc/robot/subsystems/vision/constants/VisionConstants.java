package frc.robot.subsystems.vision.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record VisionConstants(
        AprilTagFieldLayout aprilTagLayout,
        String camera0Name,
        String camera1Name,
        Transform3d robotToCamera0,
        Transform3d robotToCamera1,
        Transform3d robotToQuest,
        double maxAmbiguity,
        double maxZError,
        double linearStdDevBaseline,
        double angularStdDevBaseline,
        Matrix<N3, N1> questNavStdDevs,
        double[] cameraStdDevFactors,
        double linearStdDevMegatag2Factor,
        double angularStdDevMegatag2Factor) {}
