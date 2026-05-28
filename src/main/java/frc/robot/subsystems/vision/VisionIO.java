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

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
        public HubTagObservation[] hubTagObservations = new HubTagObservation[0];
        public FuelObservation[] fuelObservations = new FuelObservation[0];
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /** Per-tag observation from rawfiducials — horizontal angle, camera distance, and robot distance. */
    record HubTagObservation(int tagId, Rotation2d tx, double distToCamera, double distToRobot) {}

    /**
     * Detector-ready FUEL observation for classical or future ML vision. When {@code hasFieldPose} is false, autonomy
     * logs it but does not use it as a global path target.
     */
    record FuelObservation(
            double timestampSeconds,
            Pose2d fieldPose,
            boolean hasFieldPose,
            Rotation2d tx,
            double distanceMeters,
            double confidence,
            String source) {}

    /** Represents a robot pose sample used for pose estimation. */
    record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {}

    enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    default void updateInputs(VisionIOInputs inputs) {}
}
