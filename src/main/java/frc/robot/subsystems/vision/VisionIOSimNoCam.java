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
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/**
 * Lightweight IO implementation for simulation that provides "perfect" vision without simulating a camera or processing
 * images.
 */
public class VisionIOSimNoCam implements VisionIO {
    private final Supplier<Pose2d> poseSupplier;

    public VisionIOSimNoCam(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = true;

        Pose2d currentPose = poseSupplier.get();
        if (currentPose != null) {
            // Generate a single "perfect" observation based on current pose
            inputs.poseObservations = new PoseObservation[] {
                new PoseObservation(
                        Timer.getFPGATimestamp(),
                        new Pose3d(currentPose),
                        0.0, // Low ambiguity
                        2, // Simulating multi-tag
                        1.0, // Average distance
                        PoseObservationType.PHOTONVISION)
            };
            inputs.tagIds = new int[] {1, 2}; // Dummy tag IDs
        } else {
            inputs.poseObservations = new PoseObservation[0];
            inputs.tagIds = new int[0];
        }
    }
}
