package io.github.frc461.rowdy25.constants;

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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.frc461.rowdy25.util.FieldUtil;
import io.github.frc461.rowdy25.util.RotationUtil;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Meters;

/**
 * Utility class providing computed robot poses relative to field elements.
 * <p>
 * Contains static inner classes for computing robot poses at coral stations,
 * reef branches, and algae scoring locations (processor and net). All poses
 * account for the robot's physical dimensions with bumpers.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class RobotPoses {
    public static Pose2d getNearestRobotPoseAwayFromStartingLine(Pose2d currentPose) {
        return currentPose.nearest(List.of(
                new Pose2d(FieldUtil.STARTING_LINE_X_BLUE - Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters), currentPose.getY(), currentPose.getRotation()),
                new Pose2d(FieldUtil.STARTING_LINE_X_RED + Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters), currentPose.getY(), currentPose.getRotation())
        ));
    }

    /**
     * Utility methods for computing robot poses at coral stations.
     */
    public static class CoralStation {
        /**
         * Computes robot poses at each coral station tag, offset by half the robot length.
         *
         * @return List of robot poses at each coral station.
         */
        public static List<Pose2d> getRobotPosesAtEachCoralStation() {
            return FieldUtil.CoralStation.getCoralStationTagPoses().stream().map(coralStationTagPose -> coralStationTagPose.plus(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, 0, Rotation2d.kZero)
            )).toList();
        }

        /**
         * Finds the nearest robot pose at any coral station from the current pose.
         *
         * @param currentPose The robot's current pose.
         * @return The nearest coral station robot pose.
         */
        public static Pose2d getNearestRobotPoseAtCoralStation(Pose2d currentPose) {
            return currentPose.nearest(getRobotPosesAtEachCoralStation());
        }
    }

    /**
     * Utility methods for computing robot poses at and near the reef.
     */
    public static class Reef {
        /**
         * Determines whether the robot and target pose are on the same side of the reef.
         * <p>
         * Checks if the straight-line path from the robot to the target pose would intersect
         * the reef by comparing angles to reef vertices and corner distances.
         *
         * @param currentPose The robot's current pose.
         * @param targetPose The target pose to check.
         * @return True if both poses are on the same side of the reef.
         */
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
                        .map(corner -> FieldUtil.Reef.Side.getLeftVertexPoseOfNearestReef(currentPose, side).getTranslation().minus(corner.getTranslation()).getAngle())
                        .toList());
                distancesToEachVertex.addAll(robotCorners.stream()
                        .map(corner -> FieldUtil.Reef.Side.getLeftVertexPoseOfNearestReef(currentPose, side).getTranslation().getDistance(corner.getTranslation()))
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

        /**
         * Enum representing the robot's scoring offset relative to a reef branch.
         * <p>
         * Each setting defines left and right {@link Transform2d} offsets from
         * a reef tag pose for different scoring distances.
         */
        public enum RobotScoringSetting {
            /** L1 scoring offset with forward offset and lateral branch offset. */
            L1(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(3.1), Units.inchesToMeters(-10.9469731), Rotation2d.kZero),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(3.1), Units.inchesToMeters(10.9469731), Rotation2d.kZero)
            ),
            /** L2 scoring offset with forward offset and reversed rotation. */
            L2(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(6.1), Units.inchesToMeters(-6.9469731), Rotation2d.kPi),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(6.1), Units.inchesToMeters(7.4469731), Rotation2d.kPi)
            ),
            /** At-branch scoring offset. */
            AT_BRANCH(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, Units.inchesToMeters(-6.9469731), Rotation2d.kPi),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, Units.inchesToMeters(7.4469731), Rotation2d.kPi)
            ),
            /** One coral from branch scoring offset. */
            ONE_CORAL_FROM_BRANCH(
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(4.1), Units.inchesToMeters(-6.9469731), Rotation2d.kPi),
                    new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(4.1), Units.inchesToMeters(7.4469731), Rotation2d.kPi)
            );

            /** Offset transform for the left branch of the reef face. */
            final Transform2d leftOffset;
            /** Offset transform for the right branch of the reef face. */
            final Transform2d rightOffset;

            RobotScoringSetting(Transform2d leftOffset, Transform2d rightOffset) {
                this.leftOffset = leftOffset;
                this.rightOffset = rightOffset;
            }
        }

        /**
         * Computes the transform from a reef tag pose to the robot pose for algae removal.
         *
         * @param algaeIsHigh Whether targeting high or low algae.
         * @return The transform from tag to robot pose.
         */
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

        /**
         * Computes the transform from an at-branch robot pose to a near-branch robot pose.
         *
         * @param mode The scoring setting to determine offset direction.
         * @return The transform from at-branch to near-branch pose.
         */
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

        /**
         * Computes all robot poses near the reef for algae removal.
         *
         * @param algaeIsHigh Whether targeting high or low algae.
         * @param bothReefs   Whether to include both reefs or just the nearest.
         * @return List of robot poses near reef faces.
         */
        public static List<Pose2d> getRobotPosesNearReef(boolean algaeIsHigh, boolean bothReefs) {
            return FieldUtil.Reef.getReefTagPoses(bothReefs).stream().map(reefTagPose -> reefTagPose.plus(getTagToRobotPoseNearReef(algaeIsHigh))).toList();
        }

        /**
         * Gets the robot pose near a specific reef side.
         *
         * @param side The reef side to target.
         * @return The robot pose near that side.
         */
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

        /**
         * Finds the nearest robot pose near a reef from the current pose.
         *
         * @param algaeIsHigh Whether targeting high or low algae.
         * @param currentPose The robot's current pose.
         * @param bothReefs   Whether to consider both reefs.
         * @return The nearest robot pose near a reef.
         */
        public static Pose2d getNearestRobotPoseNearReef(boolean algaeIsHigh, Pose2d currentPose, boolean bothReefs) {
            return currentPose.nearest(getRobotPosesNearReef(algaeIsHigh, bothReefs));
        }

        /**
         * Finds the nearest robot pose near a reef from the current pose (considers both reefs).
         *
         * @param algaeIsHigh Whether targeting high or low algae.
         * @param currentPose The robot's current pose.
         * @return The nearest robot pose near a reef.
         */
        public static Pose2d getNearestRobotPoseNearReef(boolean algaeIsHigh, Pose2d currentPose) {
            return getNearestRobotPoseNearReef(algaeIsHigh, currentPose, true);
        }

        /**
         * Gets the robot pose for algae removal at a specific reef side.
         *
         * @param side The reef side to target.
         * @return The robot pose for algae removal.
         */
        public static Pose2d getRobotPoseAtAlgaeReef(FieldUtil.Reef.Side side) {
            if (FieldUtil.Reef.Side.algaeIsHigh(side)) {
                return FieldUtil.Reef.Side.getTag(side).pose2d.plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(2), 0, Rotation2d.kPi));
            }
            return FieldUtil.Reef.Side.getTag(side).pose2d.plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 - Units.inchesToMeters(2), 0, Rotation2d.kZero));
        }

        /**
         * Finds the nearest robot pose for algae removal at a reef.
         *
         * @param currentPose The robot's current pose.
         * @param algaeIsHigh Whether targeting high or low algae.
         * @return The nearest robot pose for algae removal.
         */
        public static Pose2d getNearestRobotPoseAtAlgaeReef(Pose2d currentPose, boolean algaeIsHigh) {
            if (algaeIsHigh) {
                return FieldUtil.Reef.getNearestReefTagPose(currentPose, true).plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + Units.inchesToMeters(2), 0, Rotation2d.kPi));
            }
            return FieldUtil.Reef.getNearestReefTagPose(currentPose, true).plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 - Units.inchesToMeters(5), 0, Rotation2d.kZero));
        }

        /**
         * Computes robot poses at all reef branches for a given scoring setting.
         * <p>
         * Generates two poses per reef tag (left and right branch).
         *
         * @param mode The scoring setting determining branch offsets.
         * @return List of robot poses at each branch.
         */
        public static List<Pose2d> getRobotPosesAtBranches(RobotScoringSetting mode) { // Where robot should be to be centered at branches (to score)
            List<Pose2d> robotPosesAtEachBranch = new ArrayList<>();
            FieldUtil.Reef.getReefTagPoses(false).forEach(reefTagPose -> {
                robotPosesAtEachBranch.add(reefTagPose.plus(mode.leftOffset));
                robotPosesAtEachBranch.add(reefTagPose.plus(mode.rightOffset));
            });
            return robotPosesAtEachBranch;
        }

        /**
         * Computes robot poses near all reef branches for a given scoring setting.
         *
         * @param mode The scoring setting determining branch offsets.
         * @return List of robot poses near each branch.
         */
        public static List<Pose2d> getRobotPosesNearBranches(RobotScoringSetting mode) {
            return getRobotPosesAtBranches(mode).stream().map(robotPoseAtBranch -> robotPoseAtBranch.plus(getRobotPoseAtToNearReef(mode))).toList();
        }

        /**
         * Gets the robot pose at a specific branch and scoring setting.
         *
         * @param mode     The scoring setting.
         * @param location The scoring location on the reef.
         * @return The robot pose at the specified branch.
         */
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

        /**
         * Gets the robot pose near a specific branch and scoring setting.
         *
         * @param mode     The scoring setting.
         * @param location The scoring location on the reef.
         * @return The robot pose near the specified branch.
         */
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

        /**
         * Finds the nearest robot pose at any branch from the current pose.
         *
         * @param mode        The scoring setting.
         * @param currentPose The robot's current pose.
         * @return The nearest robot pose at a branch.
         */
        public static Pose2d getNearestRobotPoseAtBranch(RobotScoringSetting mode, Pose2d currentPose) {
            return currentPose.nearest(getRobotPosesAtBranches(mode));
        }

        /**
         * Gets the nearest pair of branch poses (left and right) for the nearest reef face.
         *
         * @param mode        The scoring setting.
         * @param currentPose The robot's current pose.
         * @return A pair of robot poses (left, right) at the nearest reef face.
         */
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

        /**
         * Gets the nearest pair of near-branch poses for the nearest reef face.
         *
         * @param mode        The scoring setting.
         * @param currentPose The robot's current pose.
         * @return A pair of near-branch robot poses (left, right).
         */
        public static Pair<Pose2d, Pose2d> getNearestRobotPosesNearBranchPair(RobotScoringSetting mode, Pose2d currentPose) {
            Pair<Pose2d, Pose2d> atBranchPoses = getNearestRobotPosesAtBranchPair(mode, currentPose);
            Transform2d atToNear = getRobotPoseAtToNearReef(mode);
            return new Pair<>(
                    atBranchPoses.getFirst().plus(atToNear),
                    atBranchPoses.getSecond().plus(atToNear)
            );
        }
    }

    /**
     * Utility methods for computing robot poses at algae scoring locations (processor and net).
     */
    public static class AlgaeScoring {
        /**
         * Gets the robot pose at the processor for the current alliance side.
         *
         * @param currentPose The robot's current pose.
         * @return The robot pose at the alliance-side processor.
         */
        public static Pose2d getCurrentAllianceSideRobotPoseAtProcessor(Pose2d currentPose) {
            return FieldUtil.AlgaeScoring.getCurrentAllianceSideProcessorTagPose(currentPose).plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0 + 0.5, 0, Rotation2d.kZero));
        }

        /**
         * Gets the robot pose at the center of the net.
         *
         * @param currentPose The robot's current pose.
         * @return The robot pose centered at the net.
         */
        public static Pose2d getRobotPoseAtNetCenter(Pose2d currentPose) {
            return FieldUtil.AlgaeScoring.getNearestNetTagPose(currentPose).plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, 0, Rotation2d.kPi));
        }

        /**
         * Gets the innermost robot pose at the net (closest to field center).
         *
         * @param currentPose The robot's current pose.
         * @return The innermost net pose.
         */
        public static Pose2d getInnermostRobotPoseAtNet(Pose2d currentPose) {
            Pose2d robotPoseAtNetCenter = getRobotPoseAtNetCenter(currentPose);
            return new Pose2d(
                    robotPoseAtNetCenter.getX(),
                    Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red
                            ? FieldUtil.FIELD_WIDTH / 2 - Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 1.5
                            : FieldUtil.FIELD_WIDTH / 2 + Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 1.5,
                    robotPoseAtNetCenter.getRotation()
            );
        }

        /**
         * Gets the outermost robot pose at the net (farthest from field center).
         *
         * @param currentPose The robot's current pose.
         * @return The outermost net pose.
         */
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