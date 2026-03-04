/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2026 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.robot;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the perspective of the blue
 * alliance station
 */
public class FieldConstants {
    public static final FieldType fieldType = FieldType.WELDED;

    // AprilTag related constants
    public static final int aprilTagCount =
            AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    // Field dimensions
    public static final double fieldLength =
            AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double fieldWidth =
            AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

    /** Officially defined and relevant vertical lines found on the field (defined by X-axis offset) */
    public static class LinesVertical {
        public static final double center = fieldLength / 2.0;
        public static final double starting =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
        public static final double allianceZone = starting;
        public static final double hubCenter =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0;
        public static final double neutralZoneNear = center - Units.inchesToMeters(120);
        public static final double neutralZoneFar = center + Units.inchesToMeters(120);
        public static final double oppHubCenter =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0;
        public static final double oppAllianceZone =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get().getX();
    }

    /**
     * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
     *
     * <p>NOTE: The field element start and end are always left to right from the perspective of the alliance station
     */
    public static class LinesHorizontal {

        public static final double center = fieldWidth / 2.0;

        // Right of hub
        public static final double rightBumpStart = Hub.nearRightCorner.getY();
        public static final double rightBumpEnd = rightBumpStart - RightBump.width;
        public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
        public static final double rightTrenchOpenEnd = 0;

        // Left of hub
        public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
        public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
        public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
        public static final double leftTrenchOpenStart = fieldWidth;
    }

    /** Hub related constants */
    public static class Hub {

        // Dimensions
        public static final double width = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(72.0); // includes the catcher at the top
        public static final double innerWidth = Units.inchesToMeters(41.7);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        // Relevant reference points on alliance side
        public static final Translation3d topCenterPoint = new Translation3d(
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                height);
        public static final Translation3d innerCenterPoint = new Translation3d(
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                innerHeight);

        public static final Translation2d nearLeftCorner =
                new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner =
                new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d farLeftCorner =
                new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d farRightCorner =
                new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Relevant reference points on the opposite side
        public static final Translation3d oppTopCenterPoint = new Translation3d(
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                height);
        public static final Translation2d oppNearLeftCorner =
                new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppNearRightCorner =
                new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d oppFarLeftCorner =
                new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppFarRightCorner =
                new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

        // Hub faces
        public static final Pose2d nearFace =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
        public static final Pose2d farFace =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
        public static final Pose2d rightFace =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();
        public static final Pose2d leftFace =
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();
    }

    /** Left Bump related constants */
    public static class LeftBump {

        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side
        public static final Translation2d oppNearLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
        public static final Translation2d oppFarLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Right Bump related constants */
    public static class RightBump {
        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side
        public static final Translation2d oppNearLeftCorner =
                new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
        public static final Translation2d oppFarLeftCorner =
                new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Left Trench related constants */
    public static class LeftTrench {
        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft =
                new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
        public static final Translation3d openingTopRight =
                new Translation3d(LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

        // Relevant reference points on opposing side
        public static final Translation3d oppOpeningTopLeft =
                new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
        public static final Translation3d oppOpeningTopRight =
                new Translation3d(LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
    }

    public static class RightTrench {

        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft =
                new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
        public static final Translation3d openingTopRight =
                new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

        // Relevant reference points on opposing side
        public static final Translation3d oppOpeningTopLeft =
                new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
        public static final Translation3d oppOpeningTopRight =
                new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
    }

    /** Tower related constants */
    public static class Tower {
        // Dimensions
        public static final double width = Units.inchesToMeters(49.25);
        public static final double depth = Units.inchesToMeters(45.0);
        public static final double height = Units.inchesToMeters(78.25);
        public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
        public static final double frontFaceX = Units.inchesToMeters(43.51);

        public static final double uprightHeight = Units.inchesToMeters(72.1);

        // Rung heights from the floor
        public static final double lowRungHeight = Units.inchesToMeters(27.0);
        public static final double midRungHeight = Units.inchesToMeters(45.0);
        public static final double highRungHeight = Units.inchesToMeters(63.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint = new Translation2d(
                frontFaceX,
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());
        public static final Translation2d leftUpright = new Translation2d(
                frontFaceX,
                (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                        + innerOpeningWidth / 2
                        + Units.inchesToMeters(0.75));
        public static final Translation2d rightUpright = new Translation2d(
                frontFaceX,
                (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                        - innerOpeningWidth / 2
                        - Units.inchesToMeters(0.75));

        // Relevant reference points on opposing side
        public static final Translation2d oppCenterPoint = new Translation2d(
                fieldLength - frontFaceX,
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY());
        public static final Translation2d oppLeftUpright = new Translation2d(
                fieldLength - frontFaceX,
                (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                        + innerOpeningWidth / 2
                        + Units.inchesToMeters(0.75));
        public static final Translation2d oppRightUpright = new Translation2d(
                fieldLength - frontFaceX,
                (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                        - innerOpeningWidth / 2
                        - Units.inchesToMeters(0.75));
    }

    public static class Depot {
        // Dimensions
        public static final double width = Units.inchesToMeters(42.0);
        public static final double depth = Units.inchesToMeters(27.0);
        public static final double height = Units.inchesToMeters(1.125);
        public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

        // Relevant reference points on alliance side
        public static final Translation3d depotCenter =
                new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
        public static final Translation3d leftCorner =
                new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
        public static final Translation3d rightCorner =
                new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
    }

    public static class Outpost {
        // Dimensions
        public static final double width = Units.inchesToMeters(31.8);
        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
        public static final double height = Units.inchesToMeters(7.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint = new Translation2d(
                0, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
    }

    public enum FieldType {
        ANDYMARK("andymark"),
        WELDED("welded");

        private final String jsonFolder;

        FieldType(String jsonFolder) {
            this.jsonFolder = jsonFolder;
        }

        public String getJsonFolder() {
            return this.jsonFolder;
        }
    }

    public enum AprilTagLayoutType {
        OFFICIAL("2026-official"),
        NONE("2026-none");

        private final String name;
        private volatile AprilTagFieldLayout layout;
        private volatile String layoutString;

        AprilTagLayoutType(String name) {
            this.name = name;
        }

        public AprilTagFieldLayout getLayout() {
            if (layout == null) {
                synchronized (this) {
                    if (layout == null) {
                        try {
                            Path p = Path.of(
                                    Filesystem.getDeployDirectory().getPath(),
                                    "apriltags",
                                    fieldType.getJsonFolder(),
                                    name + ".json");
                            layout = new AprilTagFieldLayout(p);
                            layoutString = new ObjectMapper().writeValueAsString(layout);
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    }
                }
            }
            return layout;
        }

        public String getLayoutString() {
            if (layoutString == null) {
                getLayout();
            }
            return layoutString;
        }
    }

    /**
     * Determine whether our alliance's hub is currently active based on game data, alliance color, and match time. Uses
     * the 2026 shift schedule:
     *
     * <ul>
     *   <li>Auto: hub always active
     *   <li>Shift 1 (match time 130–105 s): active for the alliance that lost auto
     *   <li>Shift 2 (105–80 s): active for the alliance that won auto
     *   <li>Shift 3 (80–55 s): active for the alliance that lost auto
     *   <li>Shift 4 (55–30 s): active for the alliance that won auto
     *   <li>End game (≤30 s): hub always active
     * </ul>
     *
     * <p>Game data is a single character ('R' or 'B') indicating the alliance whose goal goes inactive first. That
     * alliance's goal is active in shifts 2 and 4.
     */
    public static boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as it's likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift 1 is active for the alliance that lost auto.
        // Red lost auto (redInactiveFirst) -> shift1Active for Red is false, Blue is true.
        boolean shift1Active =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        if (matchTime > 130) {
            // Transition period after auto, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    /**
     * Get the time remaining in the current hub state (active or inactive). Returns 0 if the state will not change for
     * the rest of the match.
     */
    public static double getTimeUntilHubStateChange() {
        if (!DriverStation.isTeleopEnabled()) {
            return 0.0;
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return 0.0;

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return 0.0; // Assume stable active if no data

        boolean redInactiveFirst = gameData.charAt(0) == 'R';
        boolean shift1Active =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        double matchTime = DriverStation.getMatchTime();

        // Define boundaries in descending order
        double tStart = 130.0;
        double t1 = 105.0;
        double t2 = 80.0;
        double t3 = 55.0;
        double t4 = 30.0;

        if (matchTime > tStart) {
            // Transition -> Shift 1
            // If shift 1 is same state (Active), we look further.
            // Transition is Active.
            if (shift1Active) {
                // Shift 1 is Active. Active -> Active.
                // Shift 2 is Inactive. Change happens at t1.
                return matchTime - t1;
            } else {
                // Shift 1 is Inactive. Active -> Inactive.
                // Change happens at tStart.
                return matchTime - tStart;
            }
        } else if (matchTime > t1) {
            // In Shift 1. State is shift1Active.
            // Next is Shift 2 (opposite).
            // Change happens at t1.
            return matchTime - t1;
        } else if (matchTime > t2) {
            // In Shift 2. State is !shift1Active.
            // Next is Shift 3 (shift1Active).
            // Change happens at t2.
            return matchTime - t2;
        } else if (matchTime > t3) {
            // In Shift 3. State is shift1Active.
            // Next is Shift 4 (opposite).
            // Change happens at t3.
            return matchTime - t3;
        } else if (matchTime > t4) {
            // In Shift 4. State is !shift1Active.
            // Next is Endgame (Active).
            if (!shift1Active) { // Shift 4 is Active (because shift1Active is false -> !false = true)
                // Active -> Active (Endgame). No change.
                return 0.0;
            } else {
                // Shift 4 is Inactive.
                // Inactive -> Active. Change happens at t4.
                return matchTime - t4;
            }
        } else {
            // Endgame. Always Active. No change.
            return 0.0;
        }
    }

    /** Get the hub position for the current alliance */
    public static Translation2d getHubPosition() {
        // Logic to determine alliance and return appropriate hub center
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        if (isRed) {
            // If we are Red, the target hub is the one on the Blue side (the "far" one relative to Blue driver
            // station?)
            // Wait, in recent games, you score in the hub/speaker on your opponent's side? Or your own side?
            // In 2022 (Rapid React), hub was central.
            // In 2024, speaker is on your wall.
            // The old code says:
            // Blue Hub: near wall (x ~ 4m)
            // Red Hub: far wall (x ~ 16m - 4m)
            // So each alliance has its own hub.

            // If I am Red, my hub is at RED_HUB_POSITION (far X).
            return Hub.oppTopCenterPoint.toTranslation2d();
        } else {
            // If I am Blue, my hub is at BLUE_HUB_POSITION (near X).
            return Hub.topCenterPoint.toTranslation2d();
        }
    }
}
