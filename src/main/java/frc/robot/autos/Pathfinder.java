package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotPoses;
import frc.robot.util.FieldUtil;

import java.util.List;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public final class Pathfinder {
    private static Command pathFindToPose(Pose2d targetPose, double goalEndVelocity) {
        return AutoBuilder.pathfindToPose(
                targetPose,
                Constants.AutoConstants.PATH_CONSTRAINTS,
                goalEndVelocity
        );
    }

    private static Command pathFindToPose(Pose2d targetPose) {
        return pathFindToPose(targetPose, 0.0);
    }

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

    public static Command pathFindToNearestCoralScoringLocation(RobotPoses.Reef.RobotScoringSetting mode, Pose2d currentPose) {
        return Pathfinder.pathFindToClosePose(
                RobotPoses.Reef.getNearestRobotPoseAtBranch(mode, currentPose),
                Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE,
                1.0
        );
    }

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

    public static Pose2d calculateClosePose(Pose2d targetPose, double distance, Rotation2d distanceHeading) {
        return targetPose.plus(new Transform2d(new Translation2d(-distance, distanceHeading), Rotation2d.kZero));
    }

    public static Pose2d calculateClosePose(Pose2d targetPose, double distance) {
        return calculateClosePose(targetPose, distance, Rotation2d.kZero);
    }

    private static Pose2d calculateClosePoseWithAngleScopeAndRadius(
            Pose2d currentPose,
            Pose2d targetPose,
            Rotation2d lowerAngleThreshold,
            Rotation2d upperAngleThreshold,
            double distance
    ) {
        Rotation2d distAngle = currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();

        if (inBetween(distAngle, lowerAngleThreshold, upperAngleThreshold)) {
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

    public static boolean inBetween(Rotation2d angle, Rotation2d lowerAngleThreshold, Rotation2d upperAngleThreshold) {
        if (lowerAngleThreshold.getDegrees() > upperAngleThreshold.getDegrees()) {
            return angle.getDegrees() > lowerAngleThreshold.getDegrees() || angle.getDegrees() < upperAngleThreshold.getDegrees();
        }
        return angle.getDegrees() > lowerAngleThreshold.getDegrees() && angle.getDegrees() < upperAngleThreshold.getDegrees();
    }

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
        Pose2d centerStation2Pose = FieldUtil.AprilTag.ID_12.pose2d
                    .plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero));
        Pose2d farStation1Pose = new Pose2d(Units.inchesToMeters(67.02), Units.inchesToMeters(317), FieldUtil.AprilTag.ID_13.pose2d.getRotation())
                    .plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), Constants.ROBOT_WIDTH_WITH_BUMPERS.div(2).unaryMinus().in(Meters), Rotation2d.kZero));
        Pose2d farStation2Pose = new Pose2d(Units.inchesToMeters(67.02), Units.inchesToMeters(0), FieldUtil.AprilTag.ID_12.pose2d.getRotation())
                    .plus(new Transform2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), Constants.ROBOT_WIDTH_WITH_BUMPERS.div(2).in(Meters), Rotation2d.kZero));
        Pose2d interpolatedStation1Pose = centerStation1Pose.interpolate(farStation1Pose, 0.25);
        Pose2d interpolatedStation2Pose = centerStation2Pose.interpolate(farStation2Pose, 0.25);
        Pose2d nearInterpolatedStation2Pose = farStation2Pose.plus(new Transform2d(2.0, 0, Rotation2d.fromDegrees(10)));
        System.out.println("station-1: X: " + interpolatedStation1Pose.getX() + ", Y: " + interpolatedStation1Pose.getY() + ", Angle: " + interpolatedStation1Pose.getRotation().getDegrees());
        System.out.println("station-2: X: " + interpolatedStation2Pose.getX() + ", Y: " + interpolatedStation2Pose.getY() + ", Angle: " + interpolatedStation2Pose.getRotation().getDegrees());
        System.out.println("near station-2: X: " + nearInterpolatedStation2Pose.getX() + ", Y: " + nearInterpolatedStation2Pose.getY() + ", Angle: " + nearInterpolatedStation2Pose.getRotation().getDegrees());
        System.out.println("far station-1: X: " + farStation1Pose.getX() + ", Y: " + farStation1Pose.getY() + ", Angle: " + farStation1Pose.getRotation().getDegrees());
        System.out.println("far station-2: X: " + farStation2Pose.getX() + ", Y: " + farStation2Pose.getY() + ", Angle: " + farStation2Pose.getRotation().getDegrees());

        double multiplier = 0.04 * 5 * Math.log(5 * Math.exp(2.5) + 5 - 1);
        System.out.println("Velocity function multiplier: " + multiplier);
    }
}
