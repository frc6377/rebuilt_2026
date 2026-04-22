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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
    // rawfiducials entry stride: [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
    private static final int RAW_FIDUCIALS_STRIDE = 7;
    private static final int IDX_ID = 0;
    private static final int IDX_TXNC = 1; // horizontal angle to tag center (deg, no crosshair offset)

    private final Supplier<Rotation2d> rotationSupplier;
    private final @NotNull DoubleArrayPublisher orientationPublisher;

    private final @NotNull DoubleSubscriber latencySubscriber;
    private final @NotNull DoubleSubscriber txSubscriber;
    private final @NotNull DoubleSubscriber tySubscriber;
    private final @NotNull DoubleArraySubscriber megatag1Subscriber;
    private final @NotNull DoubleArraySubscriber megatag2Subscriber;
    private final @NotNull DoubleArraySubscriber rawFiducialsSubscriber;

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name The configured name of the Limelight.
     * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
     */
    public VisionIOLimelight(@NotNull String name, Supplier<Rotation2d> rotationSupplier) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        this.rotationSupplier = rotationSupplier;
        this.orientationPublisher =
                table.getDoubleArrayTopic("robot_orientation_set").publish();
        this.latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        this.txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        this.tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        this.megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        this.megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
        this.rawFiducialsSubscriber = table.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
    }

    @Override
    public void updateInputs(@NotNull VisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = 250 > ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000);

        // Update target observation
        inputs.latestTargetObservation = new TargetObservation(
                Rotation2d.fromDegrees(this.txSubscriber.get()), Rotation2d.fromDegrees(this.tySubscriber.get()));

        // Update orientation for MegaTag 2
        this.orientationPublisher.accept(new double[] {this.rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
        NetworkTableInstance.getDefault().flush(); // Increases network traffic but recommended by Limelight

        // Read new pose observations from NetworkTables
        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var rawSample : this.megatag1Subscriber.readQueue()) {
            if (0 == rawSample.value.length) continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                    // Timestamp, based on server timestamp of publish and latency
                    rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                    // 3D pose estimate
                    parsePose(rawSample.value),

                    // Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
                    18 <= rawSample.value.length ? rawSample.value[17] : 0.0,

                    // Tag count
                    (int) rawSample.value[7],

                    // Average tag distance
                    rawSample.value[9],

                    // Observation type
                    PoseObservationType.MEGATAG_1));
        }
        for (var rawSample : this.megatag2Subscriber.readQueue()) {
            if (0 == rawSample.value.length) continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                    // Timestamp, based on server timestamp of publish and latency
                    rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

                    // 3D pose estimate
                    parsePose(rawSample.value),

                    // Ambiguity, zeroed because the pose is already disambiguated
                    0.0,

                    // Tag count
                    (int) rawSample.value[7],

                    // Average tag distance
                    rawSample.value[9],

                    // Observation type
                    PoseObservationType.MEGATAG_2));
        }

        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
        inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();

        // ── Hub-tag per-tag data from rawfiducials ──────────────────────────
        // Stride-7 format: [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
        double[] raw = this.rawFiducialsSubscriber.get();
        List<HubTagObservation> hubObs = new ArrayList<>();
        for (int i = 0; i + RAW_FIDUCIALS_STRIDE <= raw.length; i += RAW_FIDUCIALS_STRIDE) {
            int tagId = (int) raw[i + IDX_ID];
            Rotation2d tx = Rotation2d.fromDegrees(raw[i + IDX_TXNC]);
            double distToCamera = raw[i + 4];
            double distToRobot = raw[i + 5];
            hubObs.add(new HubTagObservation(tagId, tx, distToCamera, distToRobot));
        }
        inputs.hubTagObservations = hubObs.toArray(new HubTagObservation[0]);
    }

    /** Parses the 3D pose from a Limelight botpose array. */
    private static @NotNull Pose3d parsePose(double @NotNull [] rawLLArray) {
        return new Pose3d(
                rawLLArray[0],
                rawLLArray[1],
                rawLLArray[2],
                new Rotation3d(
                        Units.degreesToRadians(rawLLArray[3]),
                        Units.degreesToRadians(rawLLArray[4]),
                        Units.degreesToRadians(rawLLArray[5])));
    }
}
