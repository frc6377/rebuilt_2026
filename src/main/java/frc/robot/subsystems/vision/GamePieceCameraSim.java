package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;

public class GamePieceCameraSim {
        private static final TargetModel fuelModel = new TargetModel(Units.inchesToMeters(5.91));
        private static final Distance maxRange = Meters.of(5.0);

        private record Box(double minX, double maxX, double minY, double maxY, double minZ, double maxZ) {

                private static final double stepMeters = 0.1;

                boolean blocks(Translation3d start, Translation3d end) {
                        int samples = Math.max(1, (int) Math.ceil(start.getDistance(end) / stepMeters));
                        for (int i = 0; i <= samples; i++) {
                                if (contains(start.interpolate(end, (double) i / samples)))
                                        return true;
                        }
                        return false;
                }

                boolean contains(Translation3d point) {
                        return point.getX() >= minX
                                        && point.getX() <= maxX
                                        && point.getY() >= minY
                                        && point.getY() <= maxY
                                        && point.getZ() >= minZ
                                        && point.getZ() <= maxZ;
                }
        }

        private static final List<Box> obstacles = buildObstacles();

        private static List<Box> buildObstacles() {
                double fieldWidth = FieldConstants.fieldWidth;
                double hubHalf = FieldConstants.Hub.width / 2.0;
                double trenchHalf = FieldConstants.LeftTrench.depth / 2.0;
                double bumpHalf = FieldConstants.LeftBump.depth / 2.0;

                List<Box> boxes = new ArrayList<>();
                for (double hubX : new double[] { FieldConstants.LinesVertical.hubCenter,
                                FieldConstants.LinesVertical.oppHubCenter }) {
                        // Hub: solid box centered on the field width
                        Box hub = new Box(
                                        hubX - hubHalf,
                                        hubX + hubHalf,
                                        fieldWidth / 2.0 - hubHalf,
                                        fieldWidth / 2.0 + hubHalf,
                                        0,
                                        FieldConstants.Hub.height);

                        // Left trench: only the structure above the drive-through opening
                        Box leftTrench = new Box(
                                        hubX - trenchHalf,
                                        hubX + trenchHalf,
                                        fieldWidth - FieldConstants.LeftTrench.width,
                                        fieldWidth,
                                        FieldConstants.LeftTrench.openingHeight,
                                        FieldConstants.LeftTrench.height);

                        // Right trench: only the structure above the drive-through opening
                        Box rightTrench = new Box(
                                        hubX - trenchHalf,
                                        hubX + trenchHalf,
                                        0,
                                        FieldConstants.RightTrench.width,
                                        FieldConstants.RightTrench.openingHeight,
                                        FieldConstants.RightTrench.height);

                        // Bumps: low ramps between the hub and each trench
                        Box leftBump = new Box(
                                        hubX - bumpHalf,
                                        hubX + bumpHalf,
                                        fieldWidth / 2.0 + hubHalf,
                                        fieldWidth / 2.0 + hubHalf + FieldConstants.LeftBump.width,
                                        0,
                                        FieldConstants.LeftBump.height);

                        Box rightBump = new Box(
                                        hubX - bumpHalf,
                                        hubX + bumpHalf,
                                        fieldWidth / 2.0 - hubHalf - FieldConstants.RightBump.width,
                                        fieldWidth / 2.0 - hubHalf,
                                        0,
                                        FieldConstants.RightBump.height);

                        boxes.addAll(List.of(hub, leftTrench, rightTrench, leftBump, rightBump));
                }
                return List.copyOf(boxes);
        }

        private final PhotonCameraSim cameraSim;
        private final Transform3d robotToCamera;
        private final Supplier<Pose2d> robotPoseSupplier;

        public GamePieceCameraSim(Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier) {
                this.robotToCamera = robotToCamera;
                this.robotPoseSupplier = robotPoseSupplier;
                cameraSim = new PhotonCameraSim(
                                new PhotonCamera("gamepiece_camera"),
                                new SimCameraProperties().setCalibration(960, 720, Rotation2d.fromDegrees(70)));
                cameraSim.setMaxSightRange(maxRange.in(Meters));
        }

        public Pose3d[] getVisiblePieces() {
                Pose3d cameraPose = new Pose3d(robotPoseSupplier.get()).transformBy(robotToCamera);
                return Arrays.stream(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"))
                                .filter(fuel -> cameraSim.canSeeTargetPose(cameraPose,
                                                new VisionTargetSim(fuel, fuelModel)))
                                .filter(fuel -> hasLineOfSight(cameraPose.getTranslation(), fuel.getTranslation()))
                                .toArray(Pose3d[]::new);
        }

        public Optional<Pose3d> getClosestVisiblePiece() {
                Translation3d cameraTranslation = new Pose3d(robotPoseSupplier.get()).transformBy(robotToCamera)
                                .getTranslation();
                return Arrays.stream(getVisiblePieces())
                                .min(Comparator.comparingDouble(
                                                piece -> piece.getTranslation().getDistance(cameraTranslation)));
        }

        private static boolean hasLineOfSight(Translation3d camera, Translation3d piece) {
                for (Box obstacle : obstacles) {
                        if (obstacle.blocks(camera, piece))
                                return false;
                }
                return true;
        }
}
