package frc.robot.constants;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.FieldUtil;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Meters;

public class RobotPoses {
    public static class CoralStation {
        public static List<Pose2d> getRobotPosesAtEachCoralStation() {
            return FieldUtil.CoralStation.getCoralStationTagPoses().stream().map(coralStationTagPose -> coralStationTagPose.plus(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, 0, Rotation2d.kZero)
            )).toList();
        }

        public static Pose2d getNearestRobotPoseAtCoralStation(Pose2d currentPose) {
            return currentPose.nearest(getRobotPosesAtEachCoralStation());
        }
    }

    public static class Reef {
        public enum RobotScoringSetting {
            L1(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(8.1), Units.inchesToMeters(-5.4469731), Rotation2d.fromDegrees(15)), // TODO SHOP: TEST THIS
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(8.1), Units.inchesToMeters(8.4469731), Rotation2d.fromDegrees(-15))
            ),
            AT_BRANCH(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, Units.inchesToMeters(-5.4469731), Rotation2d.kPi),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, Units.inchesToMeters(8.4469731), Rotation2d.kPi)
            ),
            ONE_CORAL_FROM_BRANCH(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(4.1), Units.inchesToMeters(-5.4469731), Rotation2d.kPi),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(4.1), Units.inchesToMeters(8.4469731), Rotation2d.kPi)
            ),
            FACING_AWAY_ONE_CORAL_FROM_BRANCH(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(4.1), Units.inchesToMeters(-5.4469731), Rotation2d.kZero),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(4.1), Units.inchesToMeters(8.4469731), Rotation2d.kZero)
            );

            final Transform2d leftOffset;
            final Transform2d rightOffset;
            RobotScoringSetting(Transform2d leftOffset, Transform2d rightOffset) {
                this.leftOffset = leftOffset;
                this.rightOffset = rightOffset;
            }
        }

        public static List<Pose2d> getRobotPosesAtBranches(RobotScoringSetting mode) { // Where robot should be to be centered at branches (to score)
            List<Pose2d> robotPosesAtEachBranch = new ArrayList<>();
            FieldUtil.Reef.getReefTagPoses().forEach(reefTagPose -> {
                robotPosesAtEachBranch.add(reefTagPose.plus(mode.leftOffset));
                robotPosesAtEachBranch.add(reefTagPose.plus(mode.rightOffset));
            });
            return robotPosesAtEachBranch;
        }

        public static Pose2d getRobotPoseAtBranch(RobotScoringSetting mode, FieldUtil.Reef.ScoringLocation location) { // TODO SHOP: TEST THIS WITH AUTO
            return switch (location) {
                case A -> getRobotPosesAtBranches(mode).get(0);
                case B -> getRobotPosesAtBranches(mode).get(1);
                case C -> getRobotPosesAtBranches(mode).get(2);
                case D -> getRobotPosesAtBranches(mode).get(3);
                case E -> getRobotPosesAtBranches(mode).get(4);
                case F -> getRobotPosesAtBranches(mode).get(5);
                case G -> getRobotPosesAtBranches(mode).get(6);
                case H -> getRobotPosesAtBranches(mode).get(7);
                case I -> getRobotPosesAtBranches(mode).get(8);
                case J -> getRobotPosesAtBranches(mode).get(9);
                case K -> getRobotPosesAtBranches(mode).get(10);
                case L -> getRobotPosesAtBranches(mode).get(11);
            };
        }

        public static Pose2d getNearestRobotPoseAtBranch(RobotScoringSetting mode, Pose2d currentPose) {
            return currentPose.nearest(getRobotPosesAtBranches(mode));
        }

        public static Pair<Pose2d, Pose2d> getNearestRobotPosesAtBranchPair(RobotScoringSetting mode, Pose2d currentPose) {
            Pose2d nearestReefTagPose = FieldUtil.Reef.getNearestReefTagPose(currentPose);
            if (FieldUtil.Reef.getOutsideReefTags().contains(FieldUtil.Reef.getNearestReefTag(currentPose))) {
                return new Pair<>(
                        nearestReefTagPose.plus(mode.rightOffset),
                        nearestReefTagPose.plus(mode.leftOffset)
                );
            }
            return new Pair<>(
                    nearestReefTagPose.plus(mode.leftOffset),
                    nearestReefTagPose.plus(mode.rightOffset)
            );
        }

        public static Pose2d getNearestRobotPoseAtAlgaeReef(Pose2d currentPose) {
            if (FieldUtil.Reef.getAlgaeReefLevelFromTag(FieldUtil.Reef.getNearestReefTag(currentPose)) == FieldUtil.Reef.AlgaeLocation.HIGH) {
                return FieldUtil.Reef.getNearestReefTagPose(currentPose).plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, 0, Rotation2d.kPi));
            }
            return FieldUtil.Reef.getNearestReefTagPose(currentPose).plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(2), 0, Rotation2d.kZero));
        }
    }

    public static class AlgaeScoring {
        public static Pose2d getRobotPoseAtProcessor() {
            return FieldUtil.AlgaeScoring.getProcessorTagPose().plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, 0, Rotation2d.kZero));
        }

        public static Pose2d getRobotPoseAtNetCenter() {
            return FieldUtil.AlgaeScoring.getNetTagPose().plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, 0, Rotation2d.kPi));
        }

        public static Pose2d getInnermostRobotPoseAtNet() {
            return FieldUtil.AlgaeScoring.getNetTagPose().plus(new Transform2d(
                    Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0,
                    FieldUtil.AlgaeScoring.NET_SAFE_HALF_LENGTH,
                    Rotation2d.kPi
            ));
        }

        public static Pose2d getOutermostRobotPoseAtNet() {
            Pose2d robotPoseAtNetCenter = getRobotPoseAtNetCenter();
            return new Pose2d(
                    robotPoseAtNetCenter.getX(),
                    Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red
                            ? Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 1.5
                            : FieldUtil.FIELD_WIDTH - Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 1.5,
                    robotPoseAtNetCenter.getRotation()
            );
        }
    }
}
