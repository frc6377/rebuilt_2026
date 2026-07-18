package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import java.util.stream.Stream;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.util.AffineTransformation;
import org.locationtech.jts.index.strtree.STRtree;

public class PathGenerator {
    private static final PathConstraints kConstraints = new PathConstraints(
            MetersPerSecond.of(1.5),
            MetersPerSecondPerSecond.of(1.5),
            RotationsPerSecond.of(1.5),
            RotationsPerSecondPerSecond.of(1.5));
    private static final Distance kDefaultTakeoverLead = Meters.of(1.5);
    private static final double kTimeout = 3.0;
    private static final double kSpacing = 1e-3;
    private static final double kBackupM = 0.5;
    private static final double kBackupMps = 0.75;
    private static final double kTouch = 0.08;
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

    public record ZoneTrigger(Distance offset, Supplier<Command> command) {}

    public static final class ActionZone {
        final String name;
        final Envelope bounds;
        final List<ZoneTrigger> before = new ArrayList<>();
        final List<ZoneTrigger> during = new ArrayList<>();
        final List<ZoneTrigger> after = new ArrayList<>();
        Distance takeoverLead = kDefaultTakeoverLead;
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

        public ActionZone withTakeover(Distance lead, String... pathNames) {
            takeoverLead = lead;
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

        public ActionZone before(Distance m, Supplier<Command> c) {
            before.add(new ZoneTrigger(m, c));
            return this;
        }

        public ActionZone during(Distance m, Supplier<Command> c) {
            during.add(new ZoneTrigger(m, c));
            return this;
        }

        public ActionZone after(Distance m, Supplier<Command> c) {
            after.add(new ZoneTrigger(m, c));
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

        boolean near(Translation2d t, double m) {
            Envelope e = new Envelope(bounds);
            e.expandBy(m);
            Translation2d f = FlippingUtil.flipFieldPosition(t);
            return e.covers(t.getX(), t.getY()) || (flip && e.covers(f.getX(), f.getY()));
        }

        int[] intersectionSpan(PathPlannerPath path) {
            var pts = path.getAllPathPoints();
            int entry = -1, exit = -1;
            for (int i = 0; i < pts.size(); i++) {
                if (contains(pts.get(i).position)) {
                    if (entry < 0) entry = i;
                    exit = i;
                } else if (entry >= 0) {
                    break;
                }
            }
            return entry < 0 ? null : new int[] {entry, exit};
        }
    }

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
                .withTakeover(Meters.of(3), "Straight Trench L - 1")
                .flip(true));
        addActionZone(new ActionZone(
                        "RightTrench",
                        new Pose2d(hubX - half, 0, Rotation2d.kZero),
                        new Pose2d(hubX + half, FieldConstants.RightTrench.width, Rotation2d.kZero))
                .withTakeover(Meters.of(3), "Straight Trench R - 1")
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
        GoalEndState end = new GoalEndState(goalEndVelocity, target.getRotation());
        return Commands.sequence(
                backup(drive),
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
                            return useTakeover
                                    ? follow(path, constraints, drive, target, end)
                                    : AutoBuilder.followPath(path);
                        },
                        Set.of(drive)));
    }

    private static Command follow(
            PathPlannerPath path, PathConstraints c, Drive drive, Pose2d target, GoalEndState end) {
        ActionZone takeover = null;
        List<Command> parallel = new ArrayList<>();
        for (ActionZone z : actionZones) {
            if (z.intersectionSpan(path) == null) continue;
            if (takeover == null && !z.takeoverPathNames.isEmpty()) takeover = z;
            if (!z.before.isEmpty() || !z.during.isEmpty() || !z.after.isEmpty()) {
                parallel.add(zoneTriggers(z));
            }
        }
        path.preventFlipping = true;
        parallel.add(
                0, takeover == null ? AutoBuilder.followPath(path) : driveAlong(path, c, takeover, drive, target, end));
        return parallel.size() == 1 ? parallel.get(0) : Commands.parallel(parallel.toArray(Command[]::new));
    }

    private static Command zoneTriggers(ActionZone z) {
        Trigger inside = new Trigger(() -> z.contains(currentPose().getTranslation()));
        Command[] all = Stream.of(
                        z.before.stream().map(t -> Commands.waitUntil(() -> z.near(
                                        currentPose().getTranslation(),
                                        t.offset().in(Meters)))
                                .andThen(Commands.defer(t.command(), Set.of()))),
                        z.during.stream().map(t -> Commands.waitUntil(inside)
                                .andThen(Commands.defer(t.command(), Set.of()).until(inside.negate()))),
                        z.after.stream().map(t -> Commands.waitUntil(inside)
                                .andThen(Commands.waitUntil(inside.negate()))
                                .andThen(Commands.defer(t.command(), Set.of()))))
                .flatMap(s -> s)
                .toArray(Command[]::new);
        return all.length == 0 ? Commands.none() : Commands.parallel(all);
    }

    private static Command driveAlong(
            PathPlannerPath path, PathConstraints c, ActionZone zone, Drive drive, Pose2d target, GoalEndState end) {
        path.preventFlipping = true;
        Command fallback = AutoBuilder.followPath(path);
        int[] span = zone.intersectionSpan(path);
        if (span == null) return fallback;

        List<PathPoint> pathPts = path.getAllPathPoints();
        Translation2d pathEnd = pathPts.get(pathPts.size() - 1).position;
        boolean pathEndsInZone = zone.contains(pathEnd);
        Pose2d nextMajor = target;
        if (!pathEndsInZone) {
            Translation2d zoneExit = pathPts.get(span[1]).position;
            for (int i = span[1] + 1; i < pathPts.size(); i++) {
                if (pathPts.get(i).position.getDistance(zoneExit) > 0.25) {
                    nextMajor = new Pose2d(pathPts.get(i).position, target.getRotation());
                    break;
                }
            }
        }
        boolean flipSide = zone.onFlippedSide(pathPts.get(span[0]).position);
        PathPlannerPath field = selectBestTakeover(zone, flipSide, nextMajor, pathEndsInZone ? pathEnd : null);
        if (field == null) return fallback;

        double spliceDist = Math.max(0, pathPts.get(span[0]).distanceAlongPath - zone.takeoverLead.in(Meters));
        List<PathPoint> tPts = holonomic(field);
        if (tPts.size() < 2) return fallback;

        Pose2d rp = currentPose();
        Translation2d robot = rp.getTranslation();
        Rotation2d hold = rp.getRotation();
        Rotation2d takeRot = field.getGoalEndState().rotation();
        List<PathPoint> combined = new ArrayList<>();

        // Join at nearest takeover point; AD* prefix respects takeover lead when
        // outside the zone.
        Translation2d joinRef = robot;
        if (!zone.contains(robot)) {
            PathPoint lastPrefix = null;
            for (PathPoint p : pathPts) {
                if (p.distanceAlongPath > spliceDist) break;
                combined.add(pt(p.position, hold, null));
                lastPrefix = p;
            }
            joinRef = lastPrefix != null ? lastPrefix.position : pathPts.get(span[0]).position;
        }

        int join = nearestIndex(tPts, joinRef);
        int exit = pathEndsInZone ? nearestIndex(tPts, pathEnd) : betterEndpoint(tPts, takeRot, nextMajor);
        List<PathPoint> rail = along(tPts, join, exit);
        if (rail.isEmpty()) return fallback;

        var ideal = field.getIdealStartingState();
        Rotation2d joinHeading = ideal != null ? ideal.rotation() : takeRot;
        if (zone.contains(robot)) {
            combined.add(pt(robot, rot(tPts.get(join)), null));
        } else {
            combined.add(pt(rail.get(0).position, joinHeading, null));
        }
        combined.addAll(rail);
        combined = dedupe(combined);
        if (combined.size() < 2) return fallback;

        PathPoint last = combined.get(combined.size() - 1);
        List<PathPoint> prefix = List.copyOf(combined);
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            Pathfinding.ensureInitialized();
                            Pathfinding.setStartPosition(last.position);
                            Pathfinding.setGoalPosition(target.getTranslation());
                        }),
                        Commands.waitUntil(Pathfinding::isNewPathAvailable).withTimeout(kTimeout),
                        Commands.defer(
                                () -> {
                                    PathPlannerPath finish = Pathfinding.getCurrentPath(c, end);
                                    List<PathPoint> all = new ArrayList<>(prefix);
                                    if (finish != null) {
                                        all.addAll(holonomic(finish));
                                    }
                                    all = dedupe(all);
                                    if (all.size() < 2) return Commands.none();
                                    PathPlannerPath full = PathPlannerPath.fromPathPoints(all, c, end);
                                    full.preventFlipping = true;
                                    return AutoBuilder.followPath(full);
                                },
                                Set.of(drive)))
                .withName("PathfindTakeoverCombined");
    }

    /** Pick takeover with least exit→nextMajor cost: distance + 2×|Δθ|. */
    private static PathPlannerPath selectBestTakeover(
            ActionZone zone, boolean flipSide, Pose2d nextMajor, Translation2d truncateTo) {
        PathPlannerPath best = null;
        double bestCost = Double.POSITIVE_INFINITY;
        for (String name : zone.takeoverPathNames) {
            PathPlannerPath loaded = loadPath(name);
            if (loaded == null) continue;
            PathPlannerPath field = flipSide ? loaded.flipPath() : loaded;
            var pts = field.getAllPathPoints();
            if (pts.size() < 2) continue;
            Rotation2d rot = field.getGoalEndState().rotation();
            Translation2d exit = truncateTo != null
                    ? pts.get(nearestIndex(pts, truncateTo)).position
                    : pts.get(betterEndpoint(pts, rot, nextMajor)).position;
            double cost = exit.getDistance(nextMajor.getTranslation())
                    + 2.0 * Math.abs(rot.minus(nextMajor.getRotation()).getRadians());
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
                + 2.0 * Math.abs(rot.minus(next.getRotation()).getRadians());
        double c1 = pts.get(hi).position.getDistance(next.getTranslation())
                + 2.0 * Math.abs(rot.minus(next.getRotation()).getRadians());
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

        double ax = 0, ay = 0;
        boolean touching = false;
        for (Geometry cell : hits) {
            if (bumper.distance(cell) > kTouch) continue;
            touching = true;
            Envelope e = cell.getEnvelopeInternal();
            Translation2d delta = new Translation2d(
                    pose.getX() - ((e.getMinX() + e.getMaxX()) * 0.5),
                    pose.getY() - ((e.getMinY() + e.getMaxY()) * 0.5));
            if (Math.abs(delta.getX()) >= Math.abs(delta.getY())) {
                ax += Math.signum(delta.getX());
            } else {
                ay += Math.signum(delta.getY());
            }
        }
        if (!touching) return Optional.empty();
        Translation2d away = new Translation2d(ax, ay);
        return away.getNorm() < 1e-6 ? Optional.empty() : Optional.of(away.div(away.getNorm()));
    }

    private static List<PathPoint> holonomic(PathPlannerPath path) {
        var raw = path.getAllPathPoints();
        if (raw.isEmpty()) return List.of();
        var ideal = path.getIdealStartingState();
        Rotation2d start =
                ideal != null ? ideal.rotation() : path.getGoalEndState().rotation();
        Rotation2d end = path.getGoalEndState().rotation();
        double total = Math.max(raw.get(raw.size() - 1).distanceAlongPath, kSpacing);
        return raw.stream()
                .map(p -> pt(p.position, start.interpolate(end, p.distanceAlongPath / total), p.constraints))
                .toList();
    }

    private static List<PathPoint> dedupe(List<PathPoint> pts) {
        return IntStream.range(0, pts.size())
                .filter(i -> i == 0 || pts.get(i).position.getDistance(pts.get(i - 1).position) > kSpacing)
                .mapToObj(pts::get)
                .toList();
    }

    private static Rotation2d rot(PathPoint p) {
        return p.rotationTarget != null ? p.rotationTarget.rotation() : Rotation2d.kZero;
    }

    private static PathPoint pt(Translation2d t, Rotation2d r, PathConstraints c) {
        return c != null ? new PathPoint(t, new RotationTarget(0, r), c) : new PathPoint(t, new RotationTarget(0, r));
    }
}
