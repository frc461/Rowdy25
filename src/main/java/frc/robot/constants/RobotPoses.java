package frc.robot.constants;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.FieldUtil;
import frc.robot.util.RotationUtil;

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
        public static boolean sameSide(Pose2d currentPose, Pose2d targetPose) {
            List<Pose2d> robotCorners = List.of(
                    currentPose.plus(new Transform2d(
                            Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0,
                            Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0,
                            Rotation2d.kZero
                    )),
                    currentPose.plus(new Transform2d(
                            Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0,
                            -Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0,
                            Rotation2d.kZero
                    )),
                    currentPose.plus(new Transform2d(
                            -Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0,
                            Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0,
                            Rotation2d.kZero
                    )),
                    currentPose.plus(new Transform2d(
                            -Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0,
                            -Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0,
                            Rotation2d.kZero
                    ))
            );

            List<Rotation2d> anglesToEachVertex = new ArrayList<>();
            List<Double> distancesToEachVertex = new ArrayList<>();

            for (FieldUtil.Reef.Side side : FieldUtil.Reef.Side.values()) {
                anglesToEachVertex.addAll(robotCorners.stream()
                        .map(corner -> FieldUtil.Reef.Side.getLeftVertexPose(side).getTranslation().minus(corner.getTranslation()).getAngle())
                        .toList());
                distancesToEachVertex.addAll(robotCorners.stream()
                        .map(corner -> FieldUtil.Reef.Side.getLeftVertexPose(side).getTranslation().getDistance(corner.getTranslation()))
                        .toList());
            }

            Pair<Rotation2d, Rotation2d> anglesToVerticesBounds = RotationUtil.getBound(anglesToEachVertex);
            double lowestDistanceToReefCorner = distancesToEachVertex.stream().mapToDouble(Double::doubleValue).min().orElse(0.0);

            return !RotationUtil.inBetween(
                    targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle(),
                    anglesToVerticesBounds.getFirst(),
                    anglesToVerticesBounds.getSecond()
            ) && !RotationUtil.inBetween( // Safety
                    targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle(),
                    anglesToVerticesBounds.getFirst().minus(Rotation2d.fromDegrees(7.5)),
                    anglesToVerticesBounds.getSecond().plus(Rotation2d.fromDegrees(7.5))
            ) || targetPose.getTranslation().getDistance(currentPose.getTranslation()) < lowestDistanceToReefCorner;
        }

        public enum RobotScoringSetting {
            L1(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(3.1), Units.inchesToMeters(-10.9469731), Rotation2d.kZero),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(3.1), Units.inchesToMeters(10.9469731), Rotation2d.kZero)
            ),
            L2(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(6.1), Units.inchesToMeters(-6.9469731), Rotation2d.kPi),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(6.1), Units.inchesToMeters(7.4469731), Rotation2d.kPi)
            ),
            AT_BRANCH(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, Units.inchesToMeters(-6.9469731), Rotation2d.kPi),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, Units.inchesToMeters(7.4469731), Rotation2d.kPi)
            ),
            ONE_CORAL_FROM_BRANCH(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(4.1), Units.inchesToMeters(-6.9469731), Rotation2d.kPi),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(4.1), Units.inchesToMeters(7.4469731), Rotation2d.kPi)
            );

            final Transform2d leftOffset;
            final Transform2d rightOffset;
            RobotScoringSetting(Transform2d leftOffset, Transform2d rightOffset) {
                this.leftOffset = leftOffset;
                this.rightOffset = rightOffset;
            }
        }

        public static Transform2d getTagToRobotPoseNearReef(boolean algaeIsHigh) {
            if (algaeIsHigh) {
                return new Transform2d(
                        Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE,
                        0,
                        Rotation2d.kPi
                );
            }
            return new Transform2d(
                    Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE,
                    0,
                    Rotation2d.kZero
            );
        }

        public static Transform2d getRobotPoseAtToNearReef(RobotScoringSetting mode) {
            return switch (mode) {
                case L1 ->
                    new Transform2d(
                            Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE,
                            0,
                            Rotation2d.kZero
                    );
                case L2, AT_BRANCH, ONE_CORAL_FROM_BRANCH ->
                    new Transform2d(
                            -Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE,
                            0,
                            Rotation2d.kZero
                    );
            };
        }

        public static List<Pose2d> getRobotPosesNearReef(boolean algaeIsHigh, boolean bothReefs) {
            return FieldUtil.Reef.getReefTagPoses(bothReefs).stream().map(reefTagPose -> reefTagPose.plus(getTagToRobotPoseNearReef(algaeIsHigh))).toList();
        }

        public static Pose2d getRobotPoseNearReef(FieldUtil.Reef.Side side) {
            return switch (side) {
                case AB -> getRobotPosesNearReef(FieldUtil.Reef.Side.algaeIsHigh(side), false).get(0);
                case CD -> getRobotPosesNearReef(FieldUtil.Reef.Side.algaeIsHigh(side), false).get(1);
                case EF -> getRobotPosesNearReef(FieldUtil.Reef.Side.algaeIsHigh(side), false).get(2);
                case GH -> getRobotPosesNearReef(FieldUtil.Reef.Side.algaeIsHigh(side), false).get(3);
                case IJ -> getRobotPosesNearReef(FieldUtil.Reef.Side.algaeIsHigh(side), false).get(4);
                case KL -> getRobotPosesNearReef(FieldUtil.Reef.Side.algaeIsHigh(side), false).get(5);
            };
        }

        public static Pose2d getNearestRobotPoseNearReef(boolean algaeIsHigh, Pose2d currentPose, boolean bothReefs) {
            return currentPose.nearest(getRobotPosesNearReef(algaeIsHigh, bothReefs));
        }

        public static Pose2d getNearestRobotPoseNearReef(boolean algaeIsHigh, Pose2d currentPose) {
            return getNearestRobotPoseNearReef(algaeIsHigh, currentPose, false);
        }

        public static Pose2d getRobotPoseAtAlgaeReef(FieldUtil.Reef.Side side) {
            if (FieldUtil.Reef.Side.algaeIsHigh(side)) {
                return FieldUtil.Reef.Side.getTag(side).pose2d.plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(2), 0, Rotation2d.kPi));
            }
            return FieldUtil.Reef.Side.getTag(side).pose2d.plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 - Units.inchesToMeters(2), 0, Rotation2d.kZero));
        }

        public static Pose2d getNearestRobotPoseAtAlgaeReef(Pose2d currentPose, boolean algaeIsHigh) {
            if (algaeIsHigh) {
                return FieldUtil.Reef.getNearestReefTagPose(currentPose, true).plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(2), 0, Rotation2d.kPi));
            }
            return FieldUtil.Reef.getNearestReefTagPose(currentPose, true).plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 - Units.inchesToMeters(2), 0, Rotation2d.kZero));
        }

        public static List<Pose2d> getRobotPosesAtBranches(RobotScoringSetting mode) { // Where robot should be to be centered at branches (to score)
            List<Pose2d> robotPosesAtEachBranch = new ArrayList<>();
            FieldUtil.Reef.getReefTagPoses(false).forEach(reefTagPose -> {
                robotPosesAtEachBranch.add(reefTagPose.plus(mode.leftOffset));
                robotPosesAtEachBranch.add(reefTagPose.plus(mode.rightOffset));
            });
            return robotPosesAtEachBranch;
        }

        public static List<Pose2d> getRobotPosesNearBranches(RobotScoringSetting mode) {
            return getRobotPosesAtBranches(mode).stream().map(robotPoseAtBranch -> robotPoseAtBranch.plus(getRobotPoseAtToNearReef(mode))).toList();
        }

        public static Pose2d getRobotPoseAtBranch(RobotScoringSetting mode, FieldUtil.Reef.ScoringLocation location) {
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

        public static Pose2d getRobotPoseNearBranch(RobotScoringSetting mode, FieldUtil.Reef.ScoringLocation location) { // TODO SHOP: TEST THIS WITH AUTO
            return switch (location) {
                case A -> getRobotPosesNearBranches(mode).get(0);
                case B -> getRobotPosesNearBranches(mode).get(1);
                case C -> getRobotPosesNearBranches(mode).get(2);
                case D -> getRobotPosesNearBranches(mode).get(3);
                case E -> getRobotPosesNearBranches(mode).get(4);
                case F -> getRobotPosesNearBranches(mode).get(5);
                case G -> getRobotPosesNearBranches(mode).get(6);
                case H -> getRobotPosesNearBranches(mode).get(7);
                case I -> getRobotPosesNearBranches(mode).get(8);
                case J -> getRobotPosesNearBranches(mode).get(9);
                case K -> getRobotPosesNearBranches(mode).get(10);
                case L -> getRobotPosesNearBranches(mode).get(11);
            };
        }

        public static Pose2d getNearestRobotPoseAtBranch(RobotScoringSetting mode, Pose2d currentPose) {
            return currentPose.nearest(getRobotPosesAtBranches(mode));
        }

        public static Pair<Pose2d, Pose2d> getNearestRobotPosesAtBranchPair(RobotScoringSetting mode, Pose2d currentPose) {
            Pose2d nearestReefTagPose = FieldUtil.Reef.getNearestReefTagPose(currentPose, false);
            if (FieldUtil.Reef.getOutsideReefTags().contains(FieldUtil.Reef.getNearestReefTag(currentPose, false))) {
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

        public static Pair<Pose2d, Pose2d> getNearestRobotPosesNearBranchPair(RobotScoringSetting mode, Pose2d currentPose) {
            Pair<Pose2d, Pose2d> atBranchPoses = getNearestRobotPosesAtBranchPair(mode, currentPose);
            Transform2d atToNear = getRobotPoseAtToNearReef(mode);
            return new Pair<>(
                    atBranchPoses.getFirst().plus(atToNear),
                    atBranchPoses.getSecond().plus(atToNear)
            );
        }
    }

    public static class AlgaeScoring {
        public static Pose2d getCurrentAllianceSideRobotPoseAtProcessor(Pose2d currentPose) {
            return FieldUtil.AlgaeScoring.getCurrentAllianceSideProcessorTagPose(currentPose).plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + 0.5, 0, Rotation2d.kZero));
        }

        public static Pose2d getRobotPoseAtNetCenter(Pose2d currentPose) {
            return FieldUtil.AlgaeScoring.getNearestNetTagPose(currentPose).plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, 0, Rotation2d.kPi));
        }

        public static Pose2d getInnermostRobotPoseAtNet(Pose2d currentPose) {
            return FieldUtil.AlgaeScoring.getNearestNetTagPose(currentPose).plus(new Transform2d(
                    Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0,
                    FieldUtil.AlgaeScoring.NET_SAFE_HALF_LENGTH,
                    Rotation2d.kPi
            ));
        }

        public static Pose2d getOutermostRobotPoseAtNet(Pose2d currentPose) {
            Pose2d robotPoseAtNetCenter = getRobotPoseAtNetCenter(currentPose);
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
