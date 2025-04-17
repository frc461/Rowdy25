package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

import java.util.*;

public final class FieldUtil {
    public static final AprilTagFieldLayout layout2025 = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final double FIELD_LENGTH = layout2025.getFieldLength();
    public static final double FIELD_WIDTH = layout2025.getFieldWidth();
    public static final Pose3d ORIGIN = layout2025.getOrigin();

    public static boolean isInField(Pose3d pose) {
        return isInField(pose.toPose2d());
    }

    public static boolean isInField(Pose2d pose) {
        Pose2d origin2d = ORIGIN.toPose2d();
        return pose.getX() >= origin2d.getX() && pose.getX() <= origin2d.getX() + FIELD_LENGTH &&
                pose.getY() >= origin2d.getY() && pose.getY() <= origin2d.getY() + FIELD_WIDTH;
    }

    public static DriverStation.Alliance getAllianceSide(Pose2d currentPose) {
        return currentPose.getX() < FIELD_LENGTH / 2 ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red;
    }

    public enum AprilTag {
        ID_1(layout2025.getTagPose(1).orElse(new Pose3d())),
        ID_2(layout2025.getTagPose(2).orElse(new Pose3d())),
        ID_3(layout2025.getTagPose(3).orElse(new Pose3d())),
        ID_4(layout2025.getTagPose(4).orElse(new Pose3d())),
        ID_5(layout2025.getTagPose(5).orElse(new Pose3d())),
        ID_6(layout2025.getTagPose(6).orElse(new Pose3d())),
        ID_7(layout2025.getTagPose(7).orElse(new Pose3d())),
        ID_8(layout2025.getTagPose(8).orElse(new Pose3d())),
        ID_9(layout2025.getTagPose(9).orElse(new Pose3d())),
        ID_10(layout2025.getTagPose(10).orElse(new Pose3d())),
        ID_11(layout2025.getTagPose(11).orElse(new Pose3d())),
        ID_12(layout2025.getTagPose(12).orElse(new Pose3d())),
        ID_13(layout2025.getTagPose(13).orElse(new Pose3d())),
        ID_14(layout2025.getTagPose(14).orElse(new Pose3d())),
        ID_15(layout2025.getTagPose(15).orElse(new Pose3d())),
        ID_16(layout2025.getTagPose(16).orElse(new Pose3d())),
        ID_17(layout2025.getTagPose(17).orElse(new Pose3d())),
        ID_18(layout2025.getTagPose(18).orElse(new Pose3d())),
        ID_19(layout2025.getTagPose(19).orElse(new Pose3d())),
        ID_20(layout2025.getTagPose(20).orElse(new Pose3d())),
        ID_21(layout2025.getTagPose(21).orElse(new Pose3d())),
        ID_22(layout2025.getTagPose(22).orElse(new Pose3d())),
        INVALID(new Pose3d());

        public final Pose3d pose3d;
        public final Pose2d pose2d;

        AprilTag(Pose3d pose3d) {
            this.pose3d = pose3d;
            pose2d = pose3d.toPose2d();
        }

        public static final List<AprilTag> FILTER = List.of(
                ID_6, ID_7, ID_8, ID_9, ID_10, ID_11, ID_17, ID_18, ID_19, ID_20, ID_21, ID_22
        );

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

    public static final class TagManager {
        public static Map<Pose2d, AprilTag> getPosesToTags() {
            Map<Pose2d, AprilTag> posesToTags = new HashMap<>();
            Arrays.stream(AprilTag.values()).filter(tag -> tag != AprilTag.INVALID).forEach(tag -> posesToTags.put(tag.pose2d, tag));
            return posesToTags;
        }

        public static List<Pose2d> getTagLocations2d(List<AprilTag> tags) {
            List<Pose2d> tagLocations = new ArrayList<>();
            tags.forEach(tag -> tagLocations.add(tag.pose2d));
            return tagLocations;
        }
    }

    public static class CoralStation {
        public static List<AprilTag> getCoralStationTags() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ?
                    List.of(AprilTag.ID_1, AprilTag.ID_2) : List.of(AprilTag.ID_13, AprilTag.ID_12);
        }

        public static List<Pose2d> getCoralStationTagPoses() {
            return TagManager.getTagLocations2d(getCoralStationTags());
        }

        public static Pose2d getNearestCoralStationTagPose(Pose2d currentPose) {
            return currentPose.nearest(getCoralStationTagPoses());
        }

        public static AprilTag getNearestCoralStationTag(Pose2d currentPose) {
            return TagManager.getPosesToTags().getOrDefault(getNearestCoralStationTagPose(currentPose), AprilTag.INVALID);
        }
    }

    public static class Reef {
        public static final Translation2d BLUE_REEF_CENTER = AprilTag.ID_18.pose2d.getTranslation().interpolate(AprilTag.ID_21.pose2d.getTranslation(), 0.5);
        public static final Translation2d RED_REEF_CENTER = AprilTag.ID_7.pose2d.getTranslation().interpolate(AprilTag.ID_10.pose2d.getTranslation(), 0.5);
        public static final double REEF_APOTHEM = AprilTag.ID_18.pose2d.getTranslation().getDistance(AprilTag.ID_21.pose2d.getTranslation()) / 2.0;

        public static Translation2d getNearestReefCenter(Translation2d translation) {
            return translation.nearest(List.of(RED_REEF_CENTER, BLUE_REEF_CENTER));
        }

        public static Rotation2d getAngleFromNearestReefCenter(Translation2d translation) {
            return translation.minus(getNearestReefCenter(translation)).getAngle();
        }

        public static Rotation2d getAngleFromNearestReefCenter(Pose2d pose) {
            return getAngleFromNearestReefCenter(pose.getTranslation());
        }

        public enum Side {
            AB, CD, EF, GH, IJ, KL;

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

            public static boolean algaeIsHigh(Side side) {
                return getAlgaeReefLevelFromTag(getTag(side)) == AlgaeLocation.HIGH;
            }
        }

        public enum ScoringLocation {
            A, B, C, D, E, F, G, H, I, J, K, L
        }

        public enum Level {
            L1(1), L2(2), L3(3), L4(4);

            public final int level;
            Level(int level) {
                this.level = level;
            }
        }

        public static List<AprilTag> getReefTags(boolean bothReefs) {
            return bothReefs ? List.of(AprilTag.ID_7, AprilTag.ID_8, AprilTag.ID_9, AprilTag.ID_10, AprilTag.ID_11, AprilTag.ID_6,
                    AprilTag.ID_18, AprilTag.ID_17, AprilTag.ID_22, AprilTag.ID_21, AprilTag.ID_20, AprilTag.ID_19) :
                    Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red
                            ? List.of(AprilTag.ID_7, AprilTag.ID_8, AprilTag.ID_9, AprilTag.ID_10, AprilTag.ID_11, AprilTag.ID_6)
                            : List.of(AprilTag.ID_18, AprilTag.ID_17, AprilTag.ID_22, AprilTag.ID_21, AprilTag.ID_20, AprilTag.ID_19);
        }

        public static List<AprilTag> getOutsideReefTags() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red
                    ? List.of(AprilTag.ID_9, AprilTag.ID_10, AprilTag.ID_11)
                    : List.of(AprilTag.ID_22, AprilTag.ID_21, AprilTag.ID_20);
        }

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

        public static List<Pose2d> getReefTagPoses(boolean bothReefs) {
            return TagManager.getTagLocations2d(getReefTags(bothReefs));
        }

        public static Pose2d getNearestReefTagPose(Pose2d currentPose, boolean bothReefs) {
            return currentPose.nearest(getReefTagPoses(bothReefs));
        }

        public enum AlgaeLocation {
            LOW,
            HIGH
        }

        public static AprilTag getNearestReefTag(Pose2d currentPose, boolean bothReefs) {
            return TagManager.getPosesToTags().getOrDefault(getNearestReefTagPose(currentPose, bothReefs), AprilTag.INVALID);
        }

        public static AlgaeLocation getAlgaeReefLevelFromTag(AprilTag tag) {
            return switch (tag) {
                case ID_7, ID_9, ID_11, ID_18, ID_20, ID_22 -> Reef.AlgaeLocation.HIGH;
                case ID_6, ID_8, ID_10, ID_17, ID_19, ID_21  -> Reef.AlgaeLocation.LOW;
                default -> null;
            };
        }
    }

    public static class AlgaeScoring {
        public enum ScoringLocation {
            NET,
            PROCESSOR
        }

        public static final double NET_LENGTH = Units.inchesToMeters(146.50);
        public static final double NET_SAFE_HALF_LENGTH = NET_LENGTH / 2.0 - Units.inchesToMeters(9.0); // Around half of a radius of a ball

        public static List<AprilTag> getAlgaeScoringTags() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ?
                    List.of(AprilTag.ID_3, AprilTag.ID_5, AprilTag.ID_15) : List.of(AprilTag.ID_4, AprilTag.ID_14, AprilTag.ID_16);
        }

        public static List<Pose2d> getAlgaeScoringTagPoses() {
            return TagManager.getTagLocations2d(getAlgaeScoringTags());
        }

        public static Pose2d getCurrentAllianceSideProcessorTagPose(Pose2d currentPose) {
            return getAllianceSide(currentPose) == DriverStation.Alliance.Red ?
                    AprilTag.ID_3.pose2d : AprilTag.ID_16.pose2d;
        }

        public static List<Pose2d> getNetTagPoses() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ?
                    List.of(AprilTag.ID_5.pose2d, AprilTag.ID_15.pose2d) : List.of(AprilTag.ID_14.pose2d, AprilTag.ID_4.pose2d);
        }

        public static Pose2d getNearestAlgaeScoringTagPose(Pose2d currentPose) {
            return currentPose.nearest(getAlgaeScoringTagPoses());
        }

        public static Pose2d getNearestNetTagPose(Pose2d currentPose) {
            return currentPose.nearest(getNetTagPoses());
        }

        public static AprilTag getNearestAlgaeScoringTag(Pose2d currentPose) {
            return TagManager.getPosesToTags().getOrDefault(getNearestAlgaeScoringTagPose(currentPose), AprilTag.INVALID);
        }

        public static ScoringLocation getAlgaeScoringFromTag(AprilTag tag) {
            return switch (tag) {
                case ID_3, ID_16 -> ScoringLocation.PROCESSOR;
                case ID_4, ID_5, ID_14, ID_15 -> ScoringLocation.NET;
                default -> null;
            };
        }

    }
}
