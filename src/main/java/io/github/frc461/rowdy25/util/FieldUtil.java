package io.github.frc461.rowdy25.util;

/*
 * Copyright (C) 2025-present 461 Boosters FIRST, Inc. dba Westside Robotics - The Rowdy 25.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.frc461.rowdy25.constants.Constants;

import java.util.*;

/**
 * Utility class for various FRC field-related constants/measurements and constant-based calculations.
 *
 * @author Eugene Zhang, <a href="https://github.com/e500">GitHub</a>
 * @author Aneesh Terani, <a href="https://github.com/aterani">GitHub</a>
 */
public final class FieldUtil {
    /** 2025 FRC Field Layout */
    public static final AprilTagFieldLayout layout2025 = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    /** Field Length (in meters) */
    public static final double FIELD_LENGTH = layout2025.getFieldLength();
    /** Field Width (in meters) */
    public static final double FIELD_WIDTH = layout2025.getFieldWidth();
    /** Field Origin Pose */
    public static final Pose3d ORIGIN = layout2025.getOrigin();

    /**
     * Check if a given pose is within the field boundaries. Converts to {@link Pose2d} for comparison.
     *
     * @param pose The pose to compare with field bounds.
     * @return True if the pose is within the field, false otherwise.
     */
    public static boolean isInField(Pose3d pose) {
        return isInField(pose.toPose2d());
    }

    /**
     * Check if a given 2D pose is within the field boundaries.
     *
     * @param pose The 2D pose to compare with field bounds.
     * @return True if the pose is within the field, false otherwise.
     */
    public static boolean isInField(Pose2d pose) {
        Pose2d origin2d = ORIGIN.toPose2d();
        return pose.getX() >= origin2d.getX() && pose.getX() <= origin2d.getX() + FIELD_LENGTH &&
                pose.getY() >= origin2d.getY() && pose.getY() <= origin2d.getY() + FIELD_WIDTH;
    }

    /**
     * Get the alliance side based on the current pose.
     *
     * @param currentPose The current pose of the robot.
     * @return The alliance side (Blue or Red).
     */
    public static DriverStation.Alliance getAllianceSide(Pose2d currentPose) {
        return currentPose.getX() < FIELD_LENGTH / 2 ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red;
    }

    /**
     * AprilTag enum for the 2025 FRC field layout. Check <a href="https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf">this link</a> out for a visualization of the layout.
     */
    public enum AprilTag {
        /** AprilTag with ID 1 */
        ID_1(layout2025.getTagPose(1).orElse(new Pose3d())),
        /** AprilTag with ID 2 */
        ID_2(layout2025.getTagPose(2).orElse(new Pose3d())),
        /** AprilTag with ID 3 */
        ID_3(layout2025.getTagPose(3).orElse(new Pose3d())),
        /** AprilTag with ID 4 */
        ID_4(layout2025.getTagPose(4).orElse(new Pose3d())),
        /** AprilTag with ID 5 */
        ID_5(layout2025.getTagPose(5).orElse(new Pose3d())),
        /** AprilTag with ID 6 */
        ID_6(layout2025.getTagPose(6).orElse(new Pose3d())),
        /** AprilTag with ID 7 */
        ID_7(layout2025.getTagPose(7).orElse(new Pose3d())),
        /** AprilTag with ID 8 */
        ID_8(layout2025.getTagPose(8).orElse(new Pose3d())),
        /** AprilTag with ID 9 */
        ID_9(layout2025.getTagPose(9).orElse(new Pose3d())),
        /** AprilTag with ID 10 */
        ID_10(layout2025.getTagPose(10).orElse(new Pose3d())),
        /** AprilTag with ID 11 */
        ID_11(layout2025.getTagPose(11).orElse(new Pose3d())),
        /** AprilTag with ID 12 */
        ID_12(layout2025.getTagPose(12).orElse(new Pose3d())),
        /** AprilTag with ID 13 */
        ID_13(layout2025.getTagPose(13).orElse(new Pose3d())),
        /** AprilTag with ID 14 */
        ID_14(layout2025.getTagPose(14).orElse(new Pose3d())),
        /** AprilTag with ID 15 */
        ID_15(layout2025.getTagPose(15).orElse(new Pose3d())),
        /** AprilTag with ID 16 */
        ID_16(layout2025.getTagPose(16).orElse(new Pose3d())),
        /** AprilTag with ID 17 */
        ID_17(layout2025.getTagPose(17).orElse(new Pose3d())),
        /** AprilTag with ID 18 */
        ID_18(layout2025.getTagPose(18).orElse(new Pose3d())),
        /** AprilTag with ID 19 */
        ID_19(layout2025.getTagPose(19).orElse(new Pose3d())),
        /** AprilTag with ID 20 */
        ID_20(layout2025.getTagPose(20).orElse(new Pose3d())),
        /** AprilTag with ID 21 */
        ID_21(layout2025.getTagPose(21).orElse(new Pose3d())),
        /** AprilTag with ID 22 */
        ID_22(layout2025.getTagPose(22).orElse(new Pose3d())),
        /** Invalid AprilTag label */
        INVALID(new Pose3d());

        /** The 3D pose of the AprilTag on the field */
        public final Pose3d pose3d;
        /** The 2D pose of the AprilTag on the field */
        public final Pose2d pose2d;

        /** Constructor for AprilTag enum */
        AprilTag(Pose3d pose3d) {
            this.pose3d = pose3d;
            pose2d = pose3d.toPose2d();
        }

        /** A filtered list of AprilTags on the reefs relevant for PhotonVision single-tag processing */
        public static final List<AprilTag> FILTER = List.of(
                ID_6, ID_7, ID_8, ID_9, ID_10, ID_11, ID_17, ID_18, ID_19, ID_20, ID_21, ID_22
        );

        /**
         * Get the AprilTag enum corresponding to a given tag ID.
         *
         * @param tagID The ID of the AprilTag.
         * @return The corresponding AprilTag enum, or INVALID if the ID is not recognized.
         */
        public static AprilTag getTag(double tagID) {
            return switch ((int) tagID) {
                case 1 -> AprilTag.ID_1;
                case 2 -> AprilTag.ID_2;
                case 3 -> AprilTag.ID_3;
                case 4 -> AprilTag.ID_4;
                case 5 -> AprilTag.ID_5;
                case 6 -> AprilTag.ID_6;
                case 7 -> AprilTag.ID_7;
                case 8 -> AprilTag.ID_8;
                case 9 -> AprilTag.ID_9;
                case 10 -> AprilTag.ID_10;
                case 11 -> AprilTag.ID_11;
                case 12 -> AprilTag.ID_12;
                case 13 -> AprilTag.ID_13;
                case 14 -> AprilTag.ID_14;
                case 15 -> AprilTag.ID_15;
                case 16 -> AprilTag.ID_16;
                case 17 -> AprilTag.ID_17;
                case 18 -> AprilTag.ID_18;
                case 19 -> AprilTag.ID_19;
                case 20 -> AprilTag.ID_20;
                case 21 -> AprilTag.ID_21;
                case 22 -> AprilTag.ID_22;
                default -> AprilTag.INVALID;
            };
        }
    }

    /**
     * Manager class for handling AprilTag-related utilities.
     */
    public static final class TagManager {

        /**
         * Get a mapping of {@link Pose2d} to AprilTag enums for all valid tags.
         * @return Map of {@link Pose2d} to AprilTag
         */
        public static Map<Pose2d, AprilTag> getPosesToTags() {
            Map<Pose2d, AprilTag> posesToTags = new HashMap<>();
            Arrays.stream(AprilTag.values()).filter(tag -> tag != AprilTag.INVALID).forEach(tag -> posesToTags.put(tag.pose2d, tag));
            return posesToTags;
        }

        /**
         * Get a list of 2D poses for a list of AprilTags.
         *
         * @param tags List of AprilTags
         * @return List of corresponding {@link Pose2d} objects
         */
        public static List<Pose2d> getTagLocations2d(List<AprilTag> tags) {
            List<Pose2d> tagLocations = new ArrayList<>();
            tags.forEach(tag -> tagLocations.add(tag.pose2d));
            return tagLocations;
        }
    }

    /**
     * Manager class for handling coral station-related utilities.
     */
    public static class CoralStation {
        /**
         * Get the AprilTags associated with the coral stations for the current alliance.
         *
         * @return List of AprilTags for the coral stations
         */
        public static List<AprilTag> getCoralStationTags() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ?
                    List.of(AprilTag.ID_1, AprilTag.ID_2) : List.of(AprilTag.ID_13, AprilTag.ID_12);
        }

        /**
         * Get the 2D poses of the coral station tags.
         *
         * @return List of {@link Pose2d} for the coral station tags
         *
         */
        public static List<Pose2d> getCoralStationTagPoses() {
            return TagManager.getTagLocations2d(getCoralStationTags());
        }

        /**
         * Get the nearest coral station tag pose to the current pose.
         *
         * @param currentPose The current pose of the robot
         * @return The nearest coral station tag {@link Pose2d}
         */
        public static Pose2d getNearestCoralStationTagPose(Pose2d currentPose) {
            return currentPose.nearest(getCoralStationTagPoses());
        }

        /**
         * Get the nearest coral station tag to the current pose.
         *
         * @param currentPose The current pose of the robot
         * @return The nearest coral station {@link AprilTag}
         */
        public static AprilTag getNearestCoralStationTag(Pose2d currentPose) {
            return TagManager.getPosesToTags().getOrDefault(getNearestCoralStationTagPose(currentPose), AprilTag.INVALID);
        }
    }

    /**
     * Manager class for handling reef-related utilities.
     */
    public static class Reef {
        /** The center position (as an ordered pair represented by a {@link Translation2d}) of the blue reef (in meters).*/
        public static final Translation2d BLUE_REEF_CENTER = AprilTag.ID_18.pose2d.getTranslation().interpolate(AprilTag.ID_21.pose2d.getTranslation(), 0.5);
        /** The center position (as an ordered pair represented by a {@link Translation2d}) of the red reef (in meters).*/
        public static final Translation2d RED_REEF_CENTER = AprilTag.ID_7.pose2d.getTranslation().interpolate(AprilTag.ID_10.pose2d.getTranslation(), 0.5);
        /** The apothem length of the reefs (in meters).*/
        public static final double REEF_APOTHEM = AprilTag.ID_18.pose2d.getTranslation().getDistance(AprilTag.ID_21.pose2d.getTranslation()) / 2.0;

        /** Get the nearest reef center to a given translation representing a position.
         *
         * @param translation The translation to compare
         * @return The nearest reef center {@link Translation2d}
         */
        public static Translation2d getNearestReefCenter(Translation2d translation) {
            return translation.nearest(List.of(RED_REEF_CENTER, BLUE_REEF_CENTER));
        }

        /**
         * Get the angle from the nearest reef center to a given translation representing a position.
         *
         * @param translation The translation to compare
         * @return The angle from the nearest reef center to the position as a {@link Rotation2d}
         */
        public static Rotation2d getAngleFromNearestReefCenter(Translation2d translation) {
            return translation.minus(getNearestReefCenter(translation)).getAngle();
        }

        /**
         * Get the angle from the nearest reef center to a given pose representing a position.
         *
         * @param pose The pose to compare
         * @return The angle from the nearest reef center to the position as a {@link Rotation2d}
         */
        public static Rotation2d getAngleFromNearestReefCenter(Pose2d pose) {
            return getAngleFromNearestReefCenter(pose.getTranslation());
        }

        /**
         * Enum representing the sides of the reef. Side AB is the closest to the driver station, and sides proceed counter-clockwise
         */
        public enum Side {
            AB, CD, EF, GH, IJ, KL;

            /**
             * Get the left vertex pose of the nearest reef based on the current pose and specified side.
             *
             * @param currentPose The current pose of the robot
             * @param side The side of the reef
             * @return The left vertex {@link Pose2d} of the nearest reef for the specified side
             */
            public static Pose2d getLeftVertexPoseOfNearestReef(Pose2d currentPose, Side side) {
                return switch (side) {
                    case AB -> getReefCornersOfNearestReef(currentPose).get(0);
                    case CD -> getReefCornersOfNearestReef(currentPose).get(1);
                    case EF -> getReefCornersOfNearestReef(currentPose).get(2);
                    case GH -> getReefCornersOfNearestReef(currentPose).get(3);
                    case IJ -> getReefCornersOfNearestReef(currentPose).get(4);
                    case KL -> getReefCornersOfNearestReef(currentPose).get(5);
                };
            }

            /**
             * Get the left vertex pose of the reef based on the specified side.
             *
             * @param side The side of the reef
             * @return The left vertex {@link Pose2d} of the reef for the specified side
             */
            public static Pose2d getLeftVertexPose(Side side) {
                return switch (side) {
                    case AB -> getReefCorners().get(0);
                    case CD -> getReefCorners().get(1);
                    case EF -> getReefCorners().get(2);
                    case GH -> getReefCorners().get(3);
                    case IJ -> getReefCorners().get(4);
                    case KL -> getReefCorners().get(5);
                };
            }

            /**
             * Get the AprilTag associated with the specified side of the reef.
             *
             * @param side The side of the reef
             * @return The corresponding {@link AprilTag}
             */
            public static AprilTag getTag(Side side) {
                return switch (side) {
                    case AB -> getReefTags(false).get(0);
                    case CD -> getReefTags(false).get(1);
                    case EF -> getReefTags(false).get(2);
                    case GH -> getReefTags(false).get(3);
                    case IJ -> getReefTags(false).get(4);
                    case KL -> getReefTags(false).get(5);
                };
            }

            /**
             * Check if the algae reef level for the specified side is high.
             *
             * @param side The side of the reef
             * @return True if the algae reef level is high, false otherwise
             */
            public static boolean algaeIsHigh(Side side) {
                return getAlgaeReefLevelFromTag(getTag(side)) == AlgaeLocation.HIGH;
            }
        }

        /**
         * Enum representing the scoring locations on the reef. A is the left-most (looking inward toward the reef) branch/scoring location of side AB, with locations proceeding counter-clockwise.
         */
        public enum ScoringLocation {
            A, B, C, D, E, F, G, H, I, J, K, L
        }

        /**
         * Enum representing the levels of the reef.
         */
        public enum Level {
            L1(1), L2(2), L3(3), L4(4);

            /** The numerical level associated with the enum */
            public final int level;

            /** Constructor for Level enum */
            Level(int level) {
                this.level = level;
            }
        }

        /**
         * Get the AprilTags associated with the reefs for the current alliance or both alliances.
         *
         * @param bothReefs Whether to get tags for both reefs or just the current alliance's reef
         * @return List of AprilTags for the reefs
         */
        public static List<AprilTag> getReefTags(boolean bothReefs) {
            return bothReefs ? List.of(AprilTag.ID_7, AprilTag.ID_8, AprilTag.ID_9, AprilTag.ID_10, AprilTag.ID_11, AprilTag.ID_6,
                    AprilTag.ID_18, AprilTag.ID_17, AprilTag.ID_22, AprilTag.ID_21, AprilTag.ID_20, AprilTag.ID_19) :
                    Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red
                            ? List.of(AprilTag.ID_7, AprilTag.ID_8, AprilTag.ID_9, AprilTag.ID_10, AprilTag.ID_11, AprilTag.ID_6)
                            : List.of(AprilTag.ID_18, AprilTag.ID_17, AprilTag.ID_22, AprilTag.ID_21, AprilTag.ID_20, AprilTag.ID_19);
        }

        /**
         * Get the AprilTags associated with the sides facing the center of the field for the reef of the current alliance.
         *
         * @return List of AprilTags for the center-facing reef sides
         */
        public static List<AprilTag> getOutsideReefTags() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red
                    ? List.of(AprilTag.ID_9, AprilTag.ID_10, AprilTag.ID_11)
                    : List.of(AprilTag.ID_22, AprilTag.ID_21, AprilTag.ID_20);
        }

        /**
         * Get the poses of the reef corners for the current alliance.
         *
         * @return List of {@link Pose2d} for the reef corners
         */
        public static List<Pose2d> getReefCorners() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red
                    ? List.of(
                            new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(-30)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                            new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(30)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                            new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(90)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                            new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(150)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                            new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(-150)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                            new Pose2d(RED_REEF_CENTER, Rotation2d.fromDegrees(-90)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero))
                    ) : List.of(
                            new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(150)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                            new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-150)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                            new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-90)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                            new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(-30)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                            new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(30)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                            new Pose2d(BLUE_REEF_CENTER, Rotation2d.fromDegrees(90)).plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero))
                    );
        }

        /**
         * Get the poses of the reef corners for the nearest reef based on the current pose.
         *
         * @param currentPose The current pose of the robot
         * @return List of {@link Pose2d} for the reef corners of the nearest reef
         */
        public static List<Pose2d> getReefCornersOfNearestReef(Pose2d currentPose) {
            Translation2d nearestReefCenter = getNearestReefCenter(currentPose.getTranslation());
            return List.of(
                    new Pose2d(nearestReefCenter, getAllianceSide(currentPose) == DriverStation.Alliance.Red ? Rotation2d.fromDegrees(-30) : Rotation2d.fromDegrees(150))
                            .plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                    new Pose2d(nearestReefCenter, getAllianceSide(currentPose) == DriverStation.Alliance.Red ? Rotation2d.fromDegrees(30) :  Rotation2d.fromDegrees(-150))
                            .plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                    new Pose2d(nearestReefCenter, getAllianceSide(currentPose) == DriverStation.Alliance.Red ? Rotation2d.fromDegrees(90) :  Rotation2d.fromDegrees(-90))
                            .plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                    new Pose2d(nearestReefCenter, getAllianceSide(currentPose) == DriverStation.Alliance.Red ? Rotation2d.fromDegrees(150) :  Rotation2d.fromDegrees(-30))
                            .plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                    new Pose2d(nearestReefCenter, getAllianceSide(currentPose) == DriverStation.Alliance.Red ? Rotation2d.fromDegrees(-150) :  Rotation2d.fromDegrees(30))
                            .plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero)),
                    new Pose2d(nearestReefCenter, getAllianceSide(currentPose) == DriverStation.Alliance.Red ? Rotation2d.fromDegrees(-90) :  Rotation2d.fromDegrees(90))
                            .plus(new Transform2d(REEF_APOTHEM * 2 / Math.sqrt(3), 0, Rotation2d.kZero))
            );
        }

        /**
         * Get the 2D poses of the reef tags for the current alliance or both alliances.
         *
         * @param bothReefs Whether to get poses for both reefs or just the current alliance's reef
         * @return List of {@link Pose2d} for the reef tags
         */
        public static List<Pose2d> getReefTagPoses(boolean bothReefs) {
            return TagManager.getTagLocations2d(getReefTags(bothReefs));
        }

        /**
         * Get the nearest reef tag pose to the current pose.
         *
         * @param currentPose The current pose of the robot
         * @param bothReefs Whether to consider tags from both reefs or just the current alliance's reef
         * @return The nearest reef tag {@link Pose2d}
         */
        public static Pose2d getNearestReefTagPose(Pose2d currentPose, boolean bothReefs) {
            return currentPose.nearest(getReefTagPoses(bothReefs));
        }

        /**
         * Enum representing the algae reef levels.
         */
        public enum AlgaeLocation {
            LOW,
            HIGH
        }

        /**
         * Get the nearest reef tag to the current pose.
         *
         * @param currentPose The current pose of the robot
         * @param bothReefs Whether to consider tags from both reefs or just the current alliance's reef
         * @return The nearest reef {@link AprilTag}
         */
        public static AprilTag getNearestReefTag(Pose2d currentPose, boolean bothReefs) {
            return TagManager.getPosesToTags().getOrDefault(getNearestReefTagPose(currentPose, bothReefs), AprilTag.INVALID);
        }

        /**
         * Get the algae reef level associated with a given reef tag (reef side with algae).
         *
         * @param tag The reef {@link AprilTag}
         * @return The corresponding {@link AlgaeLocation}, or null if the tag is not associated with an algae level
         */
        public static AlgaeLocation getAlgaeReefLevelFromTag(AprilTag tag) {
            return switch (tag) {
                case ID_7, ID_9, ID_11, ID_18, ID_20, ID_22 -> Reef.AlgaeLocation.HIGH;
                case ID_6, ID_8, ID_10, ID_17, ID_19, ID_21  -> Reef.AlgaeLocation.LOW;
                default -> null;
            };
        }
    }

    /**
     * Manager class for handling algae scoring-related utilities.
     */
    public static class AlgaeScoring {
        /**
         * Enum representing the scoring locations for algae.
         */
        public enum ScoringLocation {
            NET,
            PROCESSOR
        }

        /** The length of the net (in meters). */
        public static final double NET_LENGTH = Units.inchesToMeters(146.50);
        /** The safe half-length of the net (in meters). */
        public static final double NET_SAFE_HALF_LENGTH = NET_LENGTH / 2.0 - Units.inchesToMeters(9.0); // Around half of a radius of a ball

        /**
         * Get the AprilTags associated with the algae scoring locations for the current alliance.
         *
         * @return List of AprilTags for the algae scoring locations
         */
        public static List<AprilTag> getAlgaeScoringTags() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ?
                    List.of(AprilTag.ID_3, AprilTag.ID_5, AprilTag.ID_15) : List.of(AprilTag.ID_4, AprilTag.ID_14, AprilTag.ID_16);
        }

        /**
         * Get the 2D poses of the algae scoring tags.
         *
         * @return List of {@link Pose2d} for the algae scoring tags
         */
        public static List<Pose2d> getAlgaeScoringTagPoses() {
            return TagManager.getTagLocations2d(getAlgaeScoringTags());
        }

        /**
         * Get the processor tag pose for the current alliance based on the current pose.
         *
         * @param currentPose The current pose of the robot
         * @return The processor tag {@link Pose2d} for the current alliance
         */
        public static Pose2d getCurrentAllianceSideProcessorTagPose(Pose2d currentPose) {
            return getAllianceSide(currentPose) == DriverStation.Alliance.Red ?
                    AprilTag.ID_3.pose2d : AprilTag.ID_16.pose2d;
        }

        /**
         * Get the net tag poses for the current alliance.
         *
         * @return List of {@link Pose2d} for the net tags
         */
        public static List<Pose2d> getNetTagPoses() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ?
                    List.of(AprilTag.ID_5.pose2d, AprilTag.ID_15.pose2d) : List.of(AprilTag.ID_14.pose2d, AprilTag.ID_4.pose2d);
        }

        /**
         * Get the nearest algae scoring tag pose to the current pose.
         *
         * @param currentPose The current pose of the robot
         * @return The nearest algae scoring tag {@link Pose2d}
         */
        public static Pose2d getNearestAlgaeScoringTagPose(Pose2d currentPose) {
            return currentPose.nearest(getAlgaeScoringTagPoses());
        }

        /**
         * Get the nearest net tag pose to the current pose.
         *
         * @param currentPose The current pose of the robot
         * @return The nearest net tag {@link Pose2d}
         */
        public static Pose2d getNearestNetTagPose(Pose2d currentPose) {
            return currentPose.nearest(getNetTagPoses());
        }

        /**
         * Get the nearest algae scoring tag to the current pose.
         *
         * @param currentPose The current pose of the robot
         * @return The nearest algae scoring {@link AprilTag}
         */
        public static AprilTag getNearestAlgaeScoringTag(Pose2d currentPose) {
            return TagManager.getPosesToTags().getOrDefault(getNearestAlgaeScoringTagPose(currentPose), AprilTag.INVALID);
        }

        /**
         * Get the scoring location associated with a given algae scoring tag.
         *
         * @param tag The algae scoring {@link AprilTag}
         * @return The corresponding {@link ScoringLocation}, or null if the tag is not associated with a scoring location
         */
        public static ScoringLocation getAlgaeScoringFromTag(AprilTag tag) {
            return switch (tag) {
                case ID_3, ID_16 -> ScoringLocation.PROCESSOR;
                case ID_4, ID_5, ID_14, ID_15 -> ScoringLocation.NET;
                default -> null;
            };
        }

    }
}
