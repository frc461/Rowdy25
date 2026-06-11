package io.github.frc461.rowdy25.autos;

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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.constants.RobotPoses;
import io.github.frc461.rowdy25.util.FieldUtil;
import io.github.frc461.rowdy25.util.RotationUtil;

import java.util.List;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/**
 * Utility class for generating autonomous pathfinding commands using PathPlanner's AutoBuilder.
 * Contains methods for dynamically navigating to specific field elements, like the nearest algae
 * or coral scoring locations, as well as logic to calculate offset poses to pathfind "close" to a target.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public final class Pathfinder {
    /**
     * Constructs a command that pathfinds to the specified target pose with a given goal end velocity.
     *
     * @param targetPose The target pose to pathfind to.
     * @param goalEndVelocity The desired velocity of the robot when it reaches the target pose.
     * @return A {@link Command} that executes the pathfinding routine.
     */
    private static Command pathFindToPose(Pose2d targetPose, double goalEndVelocity) {
        return AutoBuilder.pathfindToPose(
                targetPose,
                Constants.AutoConstants.PATH_CONSTRAINTS,
                goalEndVelocity
        );
    }

    /**
     * Constructs a command that pathfinds to the specified target pose with a default end velocity of 0.0.
     *
     * @param targetPose The target pose to pathfind to.
     * @return A {@link Command} that executes the pathfinding routine.
     */
    private static Command pathFindToPose(Pose2d targetPose) {
        return pathFindToPose(targetPose, 0.0);
    }

    /**
     * Constructs a command that pathfinds close to the nearest algae scoring location based on the robot's current pose.
     *
     * @param currentPose The current pose of the robot.
     * @return A {@link Command} for pathfinding to the calculated approach pose.
     */
    public static Command pathFindToNearestAlgaeScoringLocation(Pose2d currentPose) {
        Pose2d nearestAlgaeScoringPose = FieldUtil.AlgaeScoring.getNearestAlgaeScoringTagPose(currentPose);
        return Pathfinder.pathFindToClosePose(
                new Pose2d(
                        nearestAlgaeScoringPose.getTranslation(),
                        nearestAlgaeScoringPose.getRotation().rotateBy(Rotation2d.kPi)
                ),
                Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE,
                1.0
        );
    }

    /**
     * Constructs a command that pathfinds close to the nearest coral scoring location at the reef.
     *
     * @param mode The scoring setting representing the desired proximity/branch target.
     * @param currentPose The current pose of the robot.
     * @return A {@link Command} for pathfinding to the calculated approach pose.
     */
    public static Command pathFindToNearestCoralScoringLocation(RobotPoses.Reef.RobotScoringSetting mode, Pose2d currentPose) {
        return Pathfinder.pathFindToClosePose(
                RobotPoses.Reef.getNearestRobotPoseAtBranch(mode, currentPose),
                Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE,
                1.0
        );
    }

    /**
     * Constructs a command that pathfinds to a specified clearance distance shifted directly backward from the target pose, i.e., the distance is applied to the target pose as a transform with the opposite rotation as the target pose.
     *
     * <p>To help visualize the final calculated pose (by applying the distance as a transform with the opposite rotation as the target pose), imagine the robot is already in the target pose, but moved backward by the specified distance.</p>
     *
     * @param targetPose The absolute target pose to back away from.
     * @param distance The offset distance to maintain from the target.
     * @param goalEndVelocity The desired velocity upon arrival.
     * @return A {@link Command} that executes the pathfinding routine to the offset position.
     */
    public static Command pathFindToClosePose(
            Pose2d targetPose,
            double distance,
            double goalEndVelocity
    ) {
        return pathFindToPose(
                calculateClosePose(targetPose, distance),
                goalEndVelocity
        );
    }

    /**
     * Constructs a command that pathfinds to a specified distance away from the target pose, allowing approach from any angle. The calculating algorithm greedily optimizes by minimizing distance traveled.
     *
     * @param currentPose The current pose of the robot.
     * @param targetPose The central target pose.
     * @param distance The required radial distance from the target.
     * @return A {@link Command} for pathfinding.
     */
    public static Command pathFindToClosePose(
            Pose2d currentPose,
            Pose2d targetPose,
            double distance
    ) {
        return pathFindToClosePose(
                currentPose,
                targetPose,
                Rotation2d.fromDegrees(-180),
                Rotation2d.fromDegrees(180),
                distance
        );
    }

    /**
     * Constructs a command that pathfinds to an offset pose restricted by an angular threshold window.
     *
     * @param currentPose The current pose of the robot.
     * @param targetPose The central target pose.
     * @param lowerThreshold The CCW-most boundary of the acceptable angular range around the target.
     * @param upperThreshold The CW-most boundary of the acceptable angular range around the target.
     * @param distance The specified clearance distance to maintain from the target.
     * @return A {@link Command} for pathfinding to the valid close pose.
     */
    public static Command pathFindToClosePose(
            Pose2d currentPose,
            Pose2d targetPose,
            Rotation2d lowerThreshold,
            Rotation2d upperThreshold,
            double distance
    ) {
        return pathFindToClosePose(
                currentPose,
                targetPose,
                lowerThreshold,
                upperThreshold,
                distance,
                0.0
        );
    }

    /**
     * Constructs a command that pathfinds to an offset pose restricted by an angular threshold window, with a specified end velocity.
     * Returns an empty command if the robot is already within the specified distance.
     *
     * @param currentPose The current pose of the robot.
     * @param targetPose The central target pose.
     * @param lowerThreshold The CCW-most boundary of the acceptable angular range.
     * @param upperThreshold The CW-most boundary of the acceptable angular range.
     * @param distance The permitted distance to maintain from the target.
     * @param goalEndVelocity The desired velocity upon arrival.
     * @return A {@link Command} for pathfinding.
     */
    public static Command pathFindToClosePose(
            Pose2d currentPose,
            Pose2d targetPose,
            Rotation2d lowerThreshold,
            Rotation2d upperThreshold,
            double distance,
            double goalEndVelocity
    ) {
        if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < distance) {
            return Commands.none();
        }
        return pathFindToPose(
                calculateClosePoseWithAngleScopeAndRadius(
                        currentPose,
                        targetPose,
                        lowerThreshold,
                        upperThreshold,
                        distance
                ),
                goalEndVelocity
        );
    }

    /**
     * Calculates an offset pose shifted away from the target pose by a specified distance and along a designated angle.
     *
     * @param targetPose The original target pose.
     * @param distance The distance to offset.
     * @param distanceHeading The angular direction relative to the target's rotation along which to move the pose backward.
     * @return The offset {@link Pose2d}.
     */
    public static Pose2d calculateClosePose(Pose2d targetPose, double distance, Rotation2d distanceHeading) {
        return targetPose.plus(new Transform2d(new Translation2d(-distance, distanceHeading), Rotation2d.kZero));
    }

    /**
     * Calculates an offset pose shifted directly backward from the target pose by a specified distance.
     *
     * @param targetPose The original target pose.
     * @param distance The distance to offset directly backwards.
     * @return The offset {@link Pose2d}.
     */
    public static Pose2d calculateClosePose(Pose2d targetPose, double distance) {
        return calculateClosePose(targetPose, distance, Rotation2d.kZero);
    }

    /**
     * Computes the best approach pose on a circle around the target restricted between two angle boundaries.
     * Snaps to the boundary angles if the direct line path from the current pose is outside the scope.
     *
     * @param currentPose The current pose of the robot.
     * @param targetPose The central target pose.
     * @param lowerAngleThreshold The CCW boundary of the target approach area.
     * @param upperAngleThreshold The CW boundary of the target approach area.
     * @param distance The radius defining the distance from the target.
     * @return The calculated constrained {@link Pose2d}.
     */
    private static Pose2d calculateClosePoseWithAngleScopeAndRadius(
            Pose2d currentPose,
            Pose2d targetPose,
            Rotation2d lowerAngleThreshold,
            Rotation2d upperAngleThreshold,
            double distance
    ) {
        Rotation2d distAngle = currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();

        if (RotationUtil.inBetween(distAngle, lowerAngleThreshold, upperAngleThreshold)) {
            return new Pose2d(
                    targetPose.getTranslation().plus(new Translation2d(distance, distAngle)),
                    distAngle.rotateBy(Rotation2d.kPi)
            );
        }
        return currentPose.nearest(List.of(
                new Pose2d(
                        targetPose.getTranslation().plus(new Translation2d(distance, lowerAngleThreshold)),
                        lowerAngleThreshold.rotateBy(Rotation2d.kPi)
                ),
                new Pose2d(
                        targetPose.getTranslation().plus(new Translation2d(distance, upperAngleThreshold)),
                        upperAngleThreshold.rotateBy(Rotation2d.kPi)
                )
        ));
    }

    /**
     * Main method used for internal logic testing and verification of interpolated poses/trajectories using console.
     *
     * @param args Command-line arguments (unused).
     */
    public static void main(String[] args) {
        Constants.ALLIANCE_SUPPLIER = () -> DriverStation.Alliance.Blue;
        Constants.ROBOT_LENGTH_WITH_BUMPERS = Inches.of(38.5);
        Constants.ROBOT_WIDTH_WITH_BUMPERS = Inches.of(32.5);

        System.out.println("--------ROBOT POSES AT BRANCHES (A-L)--------");
        for (Pose2d pose : RobotPoses.Reef.getRobotPosesAtBranches(RobotPoses.Reef.RobotScoringSetting.AT_BRANCH)) {
            System.out.println("X: " + pose.getX() + ", Y: " + pose.getY() + ", Angle: " + pose.getRotation().getDegrees());
        }
        Pose2d centerStation1Pose = FieldUtil.AprilTag.ID_13.pose2d
                    .plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero));
        Pose2d centerStation2Pose = FieldUtil.AprilTag.ID_2.pose2d
                    .plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero));
        Pose2d farStation1Pose = new Pose2d(Units.inchesToMeters(67.02), Units.inchesToMeters(317), FieldUtil.AprilTag.ID_13.pose2d.getRotation())
                    .plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), Constants.ROBOT_WIDTH_WITH_BUMPERS.div(2).unaryMinus().in(Meters), Rotation2d.kZero));
        Pose2d farStation2Pose = new Pose2d(Units.inchesToMeters(67.02), Units.inchesToMeters(0), FieldUtil.AprilTag.ID_12.pose2d.getRotation())
                    .plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), Constants.ROBOT_WIDTH_WITH_BUMPERS.div(2).in(Meters), Rotation2d.kZero));
        Pose2d interpolatedStation1Pose = centerStation1Pose.interpolate(farStation1Pose, 0.25);
        Pose2d interpolatedStation2Pose = centerStation2Pose.interpolate(farStation2Pose, 0.25);
        Pose2d nearInterpolatedStation2Pose = farStation2Pose.plus(new Transform2d(2.0, 0, Rotation2d.fromDegrees(10)));
        System.out.println("center station-2: X: " + centerStation2Pose.getX() + ", Y: " + centerStation2Pose.getY() + ", Angle: " + centerStation2Pose.getRotation().getDegrees());
        System.out.println("station-1: X: " + interpolatedStation1Pose.getX() + ", Y: " + interpolatedStation1Pose.getY() + ", Angle: " + interpolatedStation1Pose.getRotation().getDegrees());
        System.out.println("station-2: X: " + interpolatedStation2Pose.getX() + ", Y: " + interpolatedStation2Pose.getY() + ", Angle: " + interpolatedStation2Pose.getRotation().getDegrees());
        System.out.println("near station-2: X: " + nearInterpolatedStation2Pose.getX() + ", Y: " + nearInterpolatedStation2Pose.getY() + ", Angle: " + nearInterpolatedStation2Pose.getRotation().getDegrees());
        System.out.println("far station-1: X: " + farStation1Pose.getX() + ", Y: " + farStation1Pose.getY() + ", Angle: " + farStation1Pose.getRotation().getDegrees());
        System.out.println("far station-2: X: " + farStation2Pose.getX() + ", Y: " + farStation2Pose.getY() + ", Angle: " + farStation2Pose.getRotation().getDegrees());
        System.out.println("red side black line X: " + (FieldUtil.FIELD_LENGTH - Units.inchesToMeters(297.5)));

        double multiplier = 0.04 * 5 * Math.log(5 * Math.exp(2.5) + 5 - 1);
        System.out.println("Velocity function multiplier: " + multiplier);
    }
}
