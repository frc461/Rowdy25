package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

import java.util.*;

public class FieldUtil {
    public static final AprilTagFieldLayout layout2025 = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
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
                    List.of(AprilTag.ID_1, AprilTag.ID_2) : List.of(AprilTag.ID_12, AprilTag.ID_13);
        }

        public static List<Pose2d> getCoralStationTagPoses() {
            return TagManager.getTagLocations2d(getCoralStationTags());
        }

        public static Pose2d getNearestCoralStationTagPose(Pose2d currentPose) {
            return currentPose.nearest(getCoralStationTagPoses());
        }
    }

    public static class Reef {
        public static final Transform2d LEFT_BRANCH_OFFSET_FROM_TAG = new Transform2d(Units.inchesToMeters(6.0), Units.inchesToMeters(-6.468878), new Rotation2d());
        public static final Transform2d RIGHT_BRANCH_OFFSET_FROM_TAG = new Transform2d(Units.inchesToMeters(6.0), Units.inchesToMeters(6.468878), new Rotation2d());

        public static List<AprilTag> getReefTags() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ?
                    List.of(
                            AprilTag.ID_6, AprilTag.ID_7, AprilTag.ID_8, AprilTag.ID_9, AprilTag.ID_10, AprilTag.ID_11
                    ) : List.of(
                            AprilTag.ID_17, AprilTag.ID_18, AprilTag.ID_19, AprilTag.ID_20, AprilTag.ID_21, AprilTag.ID_22
                    );
        }

        public static List<Pose2d> getReefTagPoses() {
            return TagManager.getTagLocations2d(getReefTags());
        }

        public static List<Pose2d> getBranchPoses() {
            List<Pose2d> branchPoses = new ArrayList<>();
            getReefTagPoses().forEach(reefTagPose -> {
                branchPoses.add(reefTagPose.plus(LEFT_BRANCH_OFFSET_FROM_TAG));
                branchPoses.add(reefTagPose.plus(RIGHT_BRANCH_OFFSET_FROM_TAG));
            });
            return branchPoses;
        }

        public static Pose2d getNearestReefTagPose(Pose2d currentPose) {
            return currentPose.nearest(getReefTagPoses());
        }

        public static Pose2d getNearestBranchPose(Pose2d currentPose) {
            return currentPose.nearest(getBranchPoses());
        }

        public enum AlgaeLocation {
            LOW,
            HIGH
        }

        public static AprilTag getNearestReefTag(Pose2d currentPose) {
            return TagManager.getPosesToTags().getOrDefault(getNearestReefTagPose(currentPose), AprilTag.INVALID);
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

        public static List<AprilTag> getAlgaeScoringTags() {
            return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ?
                    List.of(AprilTag.ID_3, AprilTag.ID_5) : List.of(AprilTag.ID_14, AprilTag.ID_16);
        }

        public static List<Pose2d> getAlgaeScoringTagPoses() {
            return TagManager.getTagLocations2d(getAlgaeScoringTags());
        }

        public static Pose2d getNearestAlgaeScoringTagPose(Pose2d currentPose) {
            return currentPose.nearest(getAlgaeScoringTagPoses());
        }

        public static AprilTag getNearestAlgaeScoringTag(Pose2d currentPose) {
            return TagManager.getPosesToTags().getOrDefault(getNearestAlgaeScoringTagPose(currentPose), AprilTag.INVALID);
        }

        public static ScoringLocation getAlgaeScoringFromTag(AprilTag tag) {
            return switch (tag) {
                case ID_3, ID_16 -> ScoringLocation.PROCESSOR;
                case ID_5, ID_14 -> ScoringLocation.NET;
                default -> null;
            };
        }

    }
}
