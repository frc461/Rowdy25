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

    public enum TagLocation {
        ID_1,
        ID_2,
        ID_3,
        ID_4,
        ID_5,
        ID_6,
        ID_7,
        ID_8,
        ID_9,
        ID_10,
        ID_11,
        ID_12,
        ID_13,
        ID_14,
        ID_15,
        ID_16,
        ID_17,
        ID_18,
        ID_19,
        ID_20,
        ID_21,
        ID_22;

        public static final Transform2d LEFT_BRANCH_OFFSET = new Transform2d(Units.inchesToMeters(-1.00), Units.inchesToMeters(-5.5), new Rotation2d());
        public static final Transform2d RIGHT_BRANCH_OFFSET = new Transform2d(Units.inchesToMeters(-1.00), Units.inchesToMeters(5.5), new Rotation2d());

        public static Pose3d getTagLocation3d(TagLocation tag) {
            return switch (tag) {
                case ID_1 -> layout2025.getTagPose(1).orElse(new Pose3d());
                case ID_2 -> layout2025.getTagPose(2).orElse(new Pose3d());
                case ID_3 -> layout2025.getTagPose(3).orElse(new Pose3d());
                case ID_4 -> layout2025.getTagPose(4).orElse(new Pose3d());
                case ID_5 -> layout2025.getTagPose(5).orElse(new Pose3d());
                case ID_6 -> layout2025.getTagPose(6).orElse(new Pose3d());
                case ID_7 -> layout2025.getTagPose(7).orElse(new Pose3d());
                case ID_8 -> layout2025.getTagPose(8).orElse(new Pose3d());
                case ID_9 -> layout2025.getTagPose(9).orElse(new Pose3d());
                case ID_10 -> layout2025.getTagPose(10).orElse(new Pose3d());
                case ID_11 -> layout2025.getTagPose(11).orElse(new Pose3d());
                case ID_12 -> layout2025.getTagPose(12).orElse(new Pose3d());
                case ID_13 -> layout2025.getTagPose(13).orElse(new Pose3d());
                case ID_14 -> layout2025.getTagPose(14).orElse(new Pose3d());
                case ID_15 -> layout2025.getTagPose(15).orElse(new Pose3d());
                case ID_16 -> layout2025.getTagPose(16).orElse(new Pose3d());
                case ID_17 -> layout2025.getTagPose(17).orElse(new Pose3d());
                case ID_18 -> layout2025.getTagPose(18).orElse(new Pose3d());
                case ID_19 -> layout2025.getTagPose(19).orElse(new Pose3d());
                case ID_20 -> layout2025.getTagPose(20).orElse(new Pose3d());
                case ID_21 -> layout2025.getTagPose(21).orElse(new Pose3d());
                case ID_22 -> layout2025.getTagPose(22).orElse(new Pose3d());
            };
        }

        public static Pose3d getTagLocation3d(double tagID) {
            return switch ((int) tagID) {
                case 1 -> layout2025.getTagPose(1).orElse(new Pose3d());
                case 2 -> layout2025.getTagPose(2).orElse(new Pose3d());
                case 3 -> layout2025.getTagPose(3).orElse(new Pose3d());
                case 4 -> layout2025.getTagPose(4).orElse(new Pose3d());
                case 5 -> layout2025.getTagPose(5).orElse(new Pose3d());
                case 6 -> layout2025.getTagPose(6).orElse(new Pose3d());
                case 7 -> layout2025.getTagPose(7).orElse(new Pose3d());
                case 8 -> layout2025.getTagPose(8).orElse(new Pose3d());
                case 9 -> layout2025.getTagPose(9).orElse(new Pose3d());
                case 10 -> layout2025.getTagPose(10).orElse(new Pose3d());
                case 11 -> layout2025.getTagPose(11).orElse(new Pose3d());
                case 12 -> layout2025.getTagPose(12).orElse(new Pose3d());
                case 13 -> layout2025.getTagPose(13).orElse(new Pose3d());
                case 14 -> layout2025.getTagPose(14).orElse(new Pose3d());
                case 15 -> layout2025.getTagPose(15).orElse(new Pose3d());
                case 16 -> layout2025.getTagPose(16).orElse(new Pose3d());
                case 17 -> layout2025.getTagPose(17).orElse(new Pose3d());
                case 18 -> layout2025.getTagPose(18).orElse(new Pose3d());
                case 19 -> layout2025.getTagPose(19).orElse(new Pose3d());
                case 20 -> layout2025.getTagPose(20).orElse(new Pose3d());
                case 21 -> layout2025.getTagPose(21).orElse(new Pose3d());
                case 22 -> layout2025.getTagPose(22).orElse(new Pose3d());
                default -> new Pose3d();
            };
        }

        public static Pose2d getTagLocation2d(TagLocation tag) {
            return getTagLocation3d(tag).toPose2d();
        }

        public static Pose2d getTagLocation2d(double tagID) {
            return getTagLocation3d(tagID).toPose2d();
        }

        public static Pose2d getNearestPoseFromGroup(Pose2d reference, Pose2d... poses) {
            Pose2d nearest = null;
            double nearestDistance = Double.MAX_VALUE;
            for (Pose2d pose : poses) {
                double distance = reference.getTranslation().getDistance(pose.getTranslation());
                if (distance < nearestDistance) {
                    nearest = pose;
                    nearestDistance = distance;
                }
            }
            return nearest;
        }

        public static Pose2d getNearestReefTagPose(Pose2d currentPose) {
            DriverStation.Alliance alliance = Constants.ALLIANCE_SUPPLIER.get();
            return alliance == DriverStation.Alliance.Red ? getNearestPoseFromGroup(
                    currentPose,
                    getTagLocation2d(TagLocation.ID_6),
                    getTagLocation2d(TagLocation.ID_7),
                    getTagLocation2d(TagLocation.ID_8),
                    getTagLocation2d(TagLocation.ID_9),
                    getTagLocation2d(TagLocation.ID_10),
                    getTagLocation2d(TagLocation.ID_11)
            ) : getNearestPoseFromGroup(
                    currentPose,
                    getTagLocation2d(TagLocation.ID_17),
                    getTagLocation2d(TagLocation.ID_18),
                    getTagLocation2d(TagLocation.ID_19),
                    getTagLocation2d(TagLocation.ID_20),
                    getTagLocation2d(TagLocation.ID_21),
                    getTagLocation2d(TagLocation.ID_22)
            );
        }

        public static Pose2d getNearestBranchPose(Pose2d currentPose) {
            DriverStation.Alliance alliance = Constants.ALLIANCE_SUPPLIER.get();
            return alliance == DriverStation.Alliance.Red ? getNearestPoseFromGroup(
                    currentPose,
                    getTagLocation2d(TagLocation.ID_6).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_6).plus(RIGHT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_7).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_7).plus(RIGHT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_8).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_8).plus(RIGHT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_9).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_9).plus(RIGHT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_10).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_10).plus(RIGHT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_11).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_11).plus(RIGHT_BRANCH_OFFSET)
            ) : getNearestPoseFromGroup(
                    currentPose,
                    getTagLocation2d(TagLocation.ID_17).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_17).plus(RIGHT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_18).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_18).plus(RIGHT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_19).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_19).plus(RIGHT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_20).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_20).plus(RIGHT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_21).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_21).plus(RIGHT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_22).plus(LEFT_BRANCH_OFFSET),
                    getTagLocation2d(TagLocation.ID_22).plus(RIGHT_BRANCH_OFFSET)
            );
        }

        public static Pose2d getNearestCoralStationTagPose(Pose2d currentPose) {
            DriverStation.Alliance alliance = Constants.ALLIANCE_SUPPLIER.get();
            return alliance == DriverStation.Alliance.Red ? getNearestPoseFromGroup(
                    currentPose,
                    getTagLocation2d(TagLocation.ID_1),
                    getTagLocation2d(TagLocation.ID_2)
            ) : getNearestPoseFromGroup(
                    currentPose,
                    getTagLocation2d(TagLocation.ID_12),
                    getTagLocation2d(TagLocation.ID_13)
            );
        }
    }
}
