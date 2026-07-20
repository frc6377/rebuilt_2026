package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.util.AffineTransformation;
import org.locationtech.jts.index.strtree.STRtree;

public class PathGenerator {
    private static final PathConstraints kConstraints = new PathConstraints(
            MetersPerSecond.of(4.0),
            MetersPerSecondPerSecond.of(0.75),
            RotationsPerSecond.of(4.0),
            RotationsPerSecondPerSecond.of(1.5));
    private static final double kTimeout = 3.0;
    private static final double kSpacing = 1e-3;
    private static final double kBackupM = 0.5;
    private static final double kBackupMps = 0.75;
    private static final double kTouch = 0.08;
    /** Max center-to-center distance for two obstacles to be treated as a close pair. */
    private static final double kCloseObstacleM = 0.45;
    /** Rotation cost weight when the destination is outside the action zone. */
    private static final double kRotCost = 2.0;
    /**
     * Rotation cost weight when the destination is inside the action zone — zones are assumed too tight to spin, so
     * takeover orientation must match the goal.
     */
    private static final double kInZoneRotCost = 20.0;
    /** Skip a finish pathfind when the takeover rail already reaches the goal. */
    private static final double kTakeoverArriveM = 0.25;

    private static final double kHalfL = Units.inchesToMeters(33.405) / 2.0;
    private static final double kHalfW = Units.inchesToMeters(37.75) / 2.0;
    private static final GeometryFactory GEOM = new GeometryFactory();
    private static final Geometry BUMPER = GEOM.createPolygon(new Coordinate[] {
        new Coordinate(kHalfL, kHalfW),
        new Coordinate(kHalfL, -kHalfW),
        new Coordinate(-kHalfL, -kHalfW),
        new Coordinate(-kHalfL, kHalfW),
        new Coordinate(kHalfL, kHalfW)
    });
    private static final STRtree obstacleTree = new STRtree();
    private static double navNodeSizeM = 0.2;
    private static boolean navGridLoaded;
    private static final List<ActionZone> actionZones = new ArrayList<>();
    private static Supplier<Pose2d> poseSupplier = AutoBuilder::getCurrentPose;

    @JsonIgnoreProperties(ignoreUnknown = true)
    private record NavGridData(double nodeSizeMeters, boolean[][] grid) {}

    public static final class ActionZone {
        final String name;
        final Envelope bounds;
        final List<String> takeoverPathNames = new ArrayList<>();
        boolean flip;

        public ActionZone(String name, Pose2d a, Pose2d b) {
            this.name = name;
            bounds = new Envelope(
                    Math.min(a.getX(), b.getX()),
                    Math.max(a.getX(), b.getX()),
                    Math.min(a.getY(), b.getY()),
                    Math.max(a.getY(), b.getY()));
        }

        public ActionZone withTakeover(String... pathNames) {
            takeoverPathNames.clear();
            for (String n : pathNames) {
                if (n != null && !n.isBlank()) takeoverPathNames.add(n);
            }
            return this;
        }

        public ActionZone flip(boolean flip) {
            this.flip = flip;
            return this;
        }

        boolean contains(Translation2d t) {
            if (bounds.covers(t.getX(), t.getY())) return true;
            if (!flip) return false;
            Translation2d f = FlippingUtil.flipFieldPosition(t);
            return bounds.covers(f.getX(), f.getY());
        }

        boolean onFlippedSide(Translation2d t) {
            return flip && !bounds.covers(t.getX(), t.getY());
        }

        /** First contiguous intersection along {@code path}, or null. */
        int[] intersectionSpan(PathPlannerPath path) {
            List<int[]> spans = intersectionSpans(path);
            return spans.isEmpty() ? null : spans.get(0);
        }

        /**
         * Every contiguous intersection of {@code path} with this zone (including flipped copies). Needed when one zone
         * is crossed more than once.
         */
        List<int[]> intersectionSpans(PathPlannerPath path) {
            List<int[]> spans = new ArrayList<>();
            var pts = path.getAllPathPoints();
            int entry = -1, exit = -1;
            for (int i = 0; i < pts.size(); i++) {
                if (contains(pts.get(i).position)) {
                    if (entry < 0) entry = i;
                    exit = i;
                } else if (entry >= 0) {
                    spans.add(new int[] {entry, exit});
                    entry = -1;
                    exit = -1;
                }
            }
            if (entry >= 0) spans.add(new int[] {entry, exit});
            return spans;
        }
    }

    /** One takeover splice opportunity along an AD* path. */
    private record Crossing(ActionZone zone, int entry, int exit) {}

    private PathGenerator() {}

    public static void setPoseSupplier(Supplier<Pose2d> s) {
        poseSupplier = s != null ? s : AutoBuilder::getCurrentPose;
    }

    private static Pose2d currentPose() {
        return poseSupplier.get();
    }

    public static void addActionZone(ActionZone zone) {
        actionZones.add(zone);
    }

    public static void ensureNavGrid() {
        if (navGridLoaded) return;
        navGridLoaded = true;
        try {
            NavGridData data = new ObjectMapper()
                    .readValue(
                            new File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json"), NavGridData.class);
            navNodeSizeM = data.nodeSizeMeters();
            boolean[][] g = data.grid();
            for (int r = 0; r < g.length; r++) {
                for (int c = 0; c < g[r].length; c++) {
                    if (!g[r][c]) continue;
                    Envelope e = new Envelope(
                            c * navNodeSizeM, (c + 1) * navNodeSizeM, r * navNodeSizeM, (r + 1) * navNodeSizeM);
                    obstacleTree.insert(e, GEOM.toGeometry(e));
                }
            }
            obstacleTree.build();
        } catch (Exception e) {
            DriverStation.reportError("PathGenerator: failed to load navgrid: " + e, e.getStackTrace());
        }
    }

    public static void registerTrenchZones() {
        ensureNavGrid();
        double hubX = FieldConstants.LinesVertical.hubCenter;
        double half = FieldConstants.LeftTrench.depth / 2.0;
        addActionZone(new ActionZone(
                        "LeftTrench",
                        new Pose2d(
                                hubX - half,
                                FieldConstants.fieldWidth - FieldConstants.LeftTrench.width,
                                Rotation2d.kZero),
                        new Pose2d(hubX + half, FieldConstants.fieldWidth, Rotation2d.kZero))
                // Filenames must match deploy/pathplanner/paths exactly (one path per heading).
                .withTakeover(
                        "Straight Trench L -1",
                        "Straight Trench L - 2",
                        "Straight Trench L - 3",
                        "Straight Trench L - 4")
                .flip(true));
        addActionZone(new ActionZone(
                        "RightTrench",
                        new Pose2d(hubX - half, 0, Rotation2d.kZero),
                        new Pose2d(hubX + half, FieldConstants.RightTrench.width, Rotation2d.kZero))
                .withTakeover(
                        "Straight Trench R - 1",
                        "Straight Trench R - 2",
                        "Straight Trench R - 3",
                        "Straight Trench R - 4")
                .flip(true));
    }

    public static Command pathfindToPose(Drive drive, Pose2d target) {
        return pathfindToPose(drive, target, kConstraints, MetersPerSecond.zero());
    }

    /** AD* pathfind with trench takeover splicing when the path crosses an action zone. */
    public static Command pathfindToPose(
            Drive drive, Pose2d target, PathConstraints constraints, LinearVelocity goalEndVelocity) {
        return pathfind(drive, target, constraints, goalEndVelocity, true).withName("PathfindToPose");
    }

    public static Command pathfindAdStar(Drive drive, Pose2d target) {
        return pathfindAdStar(drive, target, kConstraints, MetersPerSecond.zero());
    }

    /** AD* pathfind only — follows the raw Pathfinding path with no takeover splice. */
    public static Command pathfindAdStar(
            Drive drive, Pose2d target, PathConstraints constraints, LinearVelocity goalEndVelocity) {
        return pathfind(drive, target, constraints, goalEndVelocity, false).withName("PathfindAdStar");
    }

    private static Command pathfind(
            Drive drive,
            Pose2d target,
            PathConstraints constraints,
            LinearVelocity goalEndVelocity,
            boolean useTakeover) {
        return Commands.sequence(
                backup(drive), pathfindFromHere(drive, target, constraints, goalEndVelocity, useTakeover));
    }

    /** AD* from the current pose to {@code target}, with optional takeover splicing. */
    private static Command pathfindFromHere(
            Drive drive,
            Pose2d target,
            PathConstraints constraints,
            LinearVelocity goalEndVelocity,
            boolean useTakeover) {
        GoalEndState end = new GoalEndState(goalEndVelocity, target.getRotation());
        return Commands.sequence(
                Commands.runOnce(() -> {
                    Pathfinding.ensureInitialized();
                    Pathfinding.getCurrentPath(constraints, end);
                    Pose2d start = currentPose();
                    Pathfinding.setStartPosition(start.getTranslation());
                    Pathfinding.setGoalPosition(target.getTranslation());
                }),
                Commands.waitUntil(Pathfinding::isNewPathAvailable).withTimeout(kTimeout),
                Commands.defer(
                        () -> {
                            PathPlannerPath path = Pathfinding.getCurrentPath(constraints, end);
                            if (path == null || path.numPoints() < 2) return Commands.none();
                            path.preventFlipping = true;
                            return useTakeover ? follow(path, constraints, target, end) : AutoBuilder.followPath(path);
                        },
                        Set.of(drive)));
    }

    /** Takeover crossings along {@code path}, earliest entry first. */
    private static List<Crossing> takeoverCrossings(PathPlannerPath path) {
        List<Crossing> crossings = new ArrayList<>();
        for (ActionZone z : actionZones) {
            if (z.takeoverPathNames.isEmpty()) continue;
            for (int[] span : z.intersectionSpans(path)) {
                crossings.add(new Crossing(z, span[0], span[1]));
            }
        }
        crossings.sort((a, b) -> Integer.compare(a.entry(), b.entry()));
        return crossings;
    }

    private static Command follow(PathPlannerPath path, PathConstraints c, Pose2d target, GoalEndState end) {
        List<Crossing> crossings = takeoverCrossings(path);
        path.preventFlipping = true;
        if (crossings.isEmpty()) {
            return AutoBuilder.followPath(path);
        }
        PathPlannerPath merged = buildMergedTakeoverPath(path, c, target, end, crossings);
        return AutoBuilder.followPath(merged != null ? merged : path).withName("PathfindTakeoverMerged");
    }

    /**
     * One continuous path: AD* connector → takeover rail → AD* connector → … Connectors are real AD* paths between
     * segment endpoints (no point morphing).
     */
    private static PathPlannerPath buildMergedTakeoverPath(
            PathPlannerPath path, PathConstraints c, Pose2d target, GoalEndState end, List<Crossing> crossings) {
        List<PathPoint> pathPts = path.getAllPathPoints();
        if (pathPts.size() < 2 || crossings.isEmpty()) return null;

        Translation2d pathEnd = pathPts.get(pathPts.size() - 1).position;
        Pose2d rp = currentPose();
        List<PathPoint> combined = new ArrayList<>();
        Translation2d seam = rp.getTranslation();
        Rotation2d heading = rp.getRotation();

        for (int ci = 0; ci < crossings.size(); ci++) {
            Crossing crossing = crossings.get(ci);
            ActionZone zone = crossing.zone();
            int entry = crossing.entry();
            boolean stopInZone = zone.contains(pathEnd);

            boolean startInside = combined.isEmpty() && zone.contains(seam);
            Pose2d headingGoal = startInside ? new Pose2d(target.getTranslation(), heading) : target;
            boolean flipSide = zone.onFlippedSide(pathPts.get(Math.min(entry, pathPts.size() - 1)).position);
            PathPlannerPath field = selectBestTakeover(zone, flipSide, headingGoal, seam, stopInZone);
            if (field == null) continue;

            List<PathPoint> tPts = holonomic(field, c);
            if (tPts.size() < 2) continue;

            Rotation2d takeRot = field.getGoalEndState().rotation();
            int hi = tPts.size() - 1;
            int join;
            int railExit;
            if (startInside) {
                join = nearestIndex(tPts, seam);
                railExit = stopInZone ? nearestIndex(tPts, pathEnd) : betterEndpoint(tPts, takeRot, target);
            } else {
                join = tPts.get(0).position.getDistance(seam)
                                <= tPts.get(hi).position.getDistance(seam)
                        ? 0
                        : hi;
                railExit = stopInZone ? nearestIndex(tPts, pathEnd) : (join == 0 ? hi : 0);
            }
            List<PathPoint> rail = along(tPts, join, railExit);
            if (rail.isEmpty()) continue;

            Translation2d joinPos = tPts.get(join).position;
            // AD* from current seam to the start of this takeover rail.
            combined.addAll(adStarConnect(seam, heading, joinPos, takeRot, c));
            combined.addAll(rail);
            combined = new ArrayList<>(dedupe(combined));

            PathPoint last = combined.get(combined.size() - 1);
            seam = last.position;
            heading = takeRot;

            if (stopInZone) {
                if (seam.getDistance(target.getTranslation()) > kTakeoverArriveM) {
                    combined.addAll(adStarConnect(seam, heading, target.getTranslation(), takeRot, c));
                    combined = new ArrayList<>(dedupe(combined));
                }
                if (combined.size() < 2) return null;
                GoalEndState railEnd = new GoalEndState(end.velocityMPS(), takeRot);
                PathPlannerPath out = PathPlannerPath.fromPathPoints(combined, c, railEnd);
                out.preventFlipping = true;
                return out;
            }
        }

        // After the last pass-through rail, AD* to the goal.
        if (!combined.isEmpty()) {
            combined.addAll(adStarConnect(seam, heading, target.getTranslation(), target.getRotation(), c));
            combined = new ArrayList<>(dedupe(combined));
        }

        if (combined.size() < 2) return null;
        PathPlannerPath out = PathPlannerPath.fromPathPoints(combined, c, end);
        out.preventFlipping = true;
        return out;
    }

    /**
     * AD* path from {@code from} to {@code to}. Used to join takeover rails (and the goal) instead of morphing /
     * splicing waypoints by hand.
     */
    private static List<PathPoint> adStarConnect(
            Translation2d from, Rotation2d fromRot, Translation2d to, Rotation2d toRot, PathConstraints c) {
        if (from.getDistance(to) <= kTakeoverArriveM) {
            return List.of(pt(to, toRot, c));
        }

        Pathfinding.ensureInitialized();
        GoalEndState goalEnd = new GoalEndState(0.0, toRot);
        Pathfinding.setStartPosition(from);
        Pathfinding.setGoalPosition(to);
        Pathfinding.getCurrentPath(c, goalEnd);

        long deadline = System.currentTimeMillis() + (long) (kTimeout * 1000.0);
        while (!Pathfinding.isNewPathAvailable() && System.currentTimeMillis() < deadline) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        PathPlannerPath path = Pathfinding.getCurrentPath(c, goalEnd);
        if (path == null || path.numPoints() < 2) {
            return List.of(pt(from, fromRot, c), pt(to, toRot, c));
        }
        path.preventFlipping = true;
        List<PathPoint> pts = new ArrayList<>(holonomic(path, c));
        if (!pts.isEmpty()) {
            pts.set(0, pt(pts.get(0).position, fromRot, c));
            PathPoint last = pts.get(pts.size() - 1);
            pts.set(pts.size() - 1, pt(last.position, toRot, c));
        }
        return pts;
    }

    /**
     * Pick takeover by heading match + how well the ridden rail reaches the goal. Pass-through rides endpoint→endpoint;
     * in-zone stop scores the nearest rail point to the goal.
     */
    private static PathPlannerPath selectBestTakeover(
            ActionZone zone, boolean flipSide, Pose2d goal, Translation2d robot, boolean stopInZone) {
        PathPlannerPath best = null;
        double bestCost = Double.POSITIVE_INFINITY;
        double rotW = stopInZone ? kInZoneRotCost : kRotCost;
        for (String name : zone.takeoverPathNames) {
            PathPlannerPath loaded = loadPath(name);
            if (loaded == null) continue;
            PathPlannerPath field = flipSide ? loaded.flipPath() : loaded;
            var pts = field.getAllPathPoints();
            if (pts.size() < 2) continue;
            int hi = pts.size() - 1;
            Rotation2d rot = field.getGoalEndState().rotation();
            int join = pts.get(0).position.getDistance(robot)
                            <= pts.get(hi).position.getDistance(robot)
                    ? 0
                    : hi;
            int exit = stopInZone ? nearestIndex(pts, goal.getTranslation()) : (join == 0 ? hi : 0);
            Translation2d exitPos = pts.get(exit).position;
            double cost = exitPos.getDistance(goal.getTranslation())
                    + 0.25 * pts.get(join).position.getDistance(robot)
                    + rotW * Math.abs(rot.minus(goal.getRotation()).getRadians());
            if (cost < bestCost) {
                bestCost = cost;
                best = field;
            }
        }
        return best;
    }

    /** Endpoint (0 or last) with lower distance+rotation cost to next. */
    private static int betterEndpoint(List<PathPoint> pts, Rotation2d rot, Pose2d next) {
        int hi = pts.size() - 1;
        double c0 = pts.get(0).position.getDistance(next.getTranslation())
                + kRotCost * Math.abs(rot.minus(next.getRotation()).getRadians());
        double c1 = pts.get(hi).position.getDistance(next.getTranslation())
                + kRotCost * Math.abs(rot.minus(next.getRotation()).getRadians());
        return c0 <= c1 ? 0 : hi;
    }

    /** Inclusive slice from→to, reversed when approaching from the opposite side. */
    private static List<PathPoint> along(List<PathPoint> pts, int from, int to) {
        if (from <= to) return pts.subList(from, to + 1);
        List<PathPoint> out = new ArrayList<>(from - to + 1);
        for (int i = from; i >= to; i--) out.add(pts.get(i));
        return out;
    }

    private static int nearestIndex(List<PathPoint> pts, Translation2d ref) {
        int best = 0;
        for (int j = 1; j < pts.size(); j++) {
            if (pts.get(j).position.getDistance(ref) < pts.get(best).position.getDistance(ref)) best = j;
        }
        return best;
    }

    private static PathPlannerPath loadPath(String name) {
        try {
            return PathPlannerPath.fromPathFile(name);
        } catch (Exception e) {
            DriverStation.reportError(
                    "PathGenerator: failed to load takeover path '" + name + "': " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    private static Command backup(Drive drive) {
        return Commands.defer(
                        () -> {
                            ensureNavGrid();
                            Pose2d start = currentPose();
                            Optional<Translation2d> awayOpt = awayFromObstacles(start);
                            if (awayOpt.isEmpty()) {
                                return Commands.none();
                            }
                            Translation2d away = awayOpt.get();
                            Translation2d startT = start.getTranslation();
                            return Commands.run(
                                            () -> drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                                    away.getX() * kBackupMps,
                                                    away.getY() * kBackupMps,
                                                    0,
                                                    currentPose().getRotation())),
                                            drive)
                                    .until(() ->
                                            startT.getDistance(currentPose().getTranslation()) >= kBackupM)
                                    .finallyDo(interrupted -> drive.stop())
                                    .withTimeout(kBackupM / kBackupMps + 1);
                        },
                        Set.of(drive))
                .withName("ObstacleBackup");
    }

    /** Unit field vector away from touched navgrid faces; empty if bumpers are clear. */
    private static Optional<Translation2d> awayFromObstacles(Pose2d pose) {
        AffineTransformation tx = new AffineTransformation();
        tx.rotate(pose.getRotation().getRadians());
        tx.translate(pose.getX(), pose.getY());
        Geometry bumper = tx.transform(BUMPER);
        Envelope search = new Envelope(bumper.getEnvelopeInternal());
        search.expandBy(kTouch + navNodeSizeM);
        @SuppressWarnings("unchecked")
        List<Geometry> hits = obstacleTree.query(search);
        if (hits.isEmpty()) return Optional.empty();

        List<Translation2d> centers = new ArrayList<>();
        double ax = 0, ay = 0;
        for (Geometry cell : hits) {
            if (bumper.distance(cell) > kTouch) continue;
            Envelope e = cell.getEnvelopeInternal();
            Translation2d center =
                    new Translation2d((e.getMinX() + e.getMaxX()) * 0.5, (e.getMinY() + e.getMaxY()) * 0.5);
            centers.add(center);
            Translation2d delta = new Translation2d(pose.getX() - center.getX(), pose.getY() - center.getY());
            if (Math.abs(delta.getX()) >= Math.abs(delta.getY())) {
                ax += Math.signum(delta.getX());
            } else {
                ay += Math.signum(delta.getY());
            }
        }
        if (centers.isEmpty()) return Optional.empty();

        // Two close obstacles: flee from their midpoint (equal distance), or along the
        // gap bisector when the robot sits between them and axis pushes cancel.
        Optional<Translation2d> pairAway = awayFromClosePair(pose.getTranslation(), centers);
        if (pairAway.isPresent()) return pairAway;

        Translation2d away = new Translation2d(ax, ay);
        return away.getNorm() < 1e-6 ? Optional.empty() : Optional.of(away.div(away.getNorm()));
    }

    /**
     * If the two nearest touching obstacles are close, return a unit vector away from their midpoint. When the robot is
     * on that midpoint (e.g. pinched in a corridor), use the perpendicular bisector.
     */
    private static Optional<Translation2d> awayFromClosePair(Translation2d robot, List<Translation2d> centers) {
        if (centers.size() < 2) return Optional.empty();

        int i0 = 0, i1 = 1;
        double bestDist = Double.POSITIVE_INFINITY;
        for (int i = 0; i < centers.size(); i++) {
            for (int j = i + 1; j < centers.size(); j++) {
                double d = centers.get(i).getDistance(centers.get(j));
                if (d < bestDist) {
                    bestDist = d;
                    i0 = i;
                    i1 = j;
                }
            }
        }
        if (bestDist > kCloseObstacleM) return Optional.empty();

        Translation2d a = centers.get(i0);
        Translation2d b = centers.get(i1);
        Translation2d mid = a.plus(b).times(0.5);
        Translation2d fromMid = robot.minus(mid);
        if (fromMid.getNorm() > 1e-3) {
            return Optional.of(fromMid.div(fromMid.getNorm()));
        }

        // Robot is between the pair — slide along the gap (perpendicular to A→B).
        Translation2d ab = b.minus(a);
        if (ab.getNorm() < 1e-6) return Optional.empty();
        Translation2d alongGap = new Translation2d(-ab.getY(), ab.getX()).div(ab.getNorm());
        // Prefer the side closer to field center so we don't drive into the wall edge.
        Translation2d fieldCenter =
                new Translation2d(FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth / 2.0);
        if (alongGap.dot(fieldCenter.minus(robot)) < 0) {
            alongGap = alongGap.times(-1.0);
        }
        return Optional.of(alongGap);
    }

    /** Holonomic resample; {@code constraints} overrides per-point file constraints when non-null. */
    private static List<PathPoint> holonomic(PathPlannerPath path, PathConstraints constraints) {
        var raw = path.getAllPathPoints();
        if (raw.isEmpty()) return List.of();
        var ideal = path.getIdealStartingState();
        Rotation2d start =
                ideal != null ? ideal.rotation() : path.getGoalEndState().rotation();
        Rotation2d end = path.getGoalEndState().rotation();
        double total = Math.max(raw.get(raw.size() - 1).distanceAlongPath, kSpacing);
        return raw.stream()
                .map(p -> pt(
                        p.position,
                        start.interpolate(end, p.distanceAlongPath / total),
                        constraints != null ? constraints : p.constraints))
                .toList();
    }

    private static List<PathPoint> dedupe(List<PathPoint> pts) {
        return IntStream.range(0, pts.size())
                .filter(i -> i == 0 || pts.get(i).position.getDistance(pts.get(i - 1).position) > kSpacing)
                .mapToObj(pts::get)
                .toList();
    }

    private static PathPoint pt(Translation2d t, Rotation2d r, PathConstraints c) {
        return c != null ? new PathPoint(t, new RotationTarget(0, r), c) : new PathPoint(t, new RotationTarget(0, r));
    }
}
