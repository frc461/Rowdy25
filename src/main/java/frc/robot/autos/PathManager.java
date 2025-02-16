package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.routines.AutoEventLooper;
import frc.robot.constants.Constants;
import frc.robot.util.FieldUtil;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.List;

public final class PathManager {
    // ONE START
    public static PathPlannerPath ONE_START_TO_A;
    public static PathPlannerPath ONE_START_TO_B;
    public static PathPlannerPath ONE_START_TO_C;
    public static PathPlannerPath ONE_START_TO_D;
    public static PathPlannerPath ONE_START_TO_E;
    public static PathPlannerPath ONE_START_TO_F;
    public static PathPlannerPath ONE_START_TO_G;
    public static PathPlannerPath ONE_START_TO_H;
    public static PathPlannerPath ONE_START_TO_I;
    public static PathPlannerPath ONE_START_TO_J;
    public static PathPlannerPath ONE_START_TO_K;
    public static PathPlannerPath ONE_START_TO_L;

    // TWO START
    public static PathPlannerPath TWO_START_TO_A;
    public static PathPlannerPath TWO_START_TO_B;
    public static PathPlannerPath TWO_START_TO_C;
    public static PathPlannerPath TWO_START_TO_D;
    public static PathPlannerPath TWO_START_TO_E;
    public static PathPlannerPath TWO_START_TO_F;
    public static PathPlannerPath TWO_START_TO_G;
    public static PathPlannerPath TWO_START_TO_H;
    public static PathPlannerPath TWO_START_TO_I;
    public static PathPlannerPath TWO_START_TO_J;
    public static PathPlannerPath TWO_START_TO_K;
    public static PathPlannerPath TWO_START_TO_L;

    // THREE START 
    public static PathPlannerPath THREE_START_TO_A;
    public static PathPlannerPath THREE_START_TO_B;
    public static PathPlannerPath THREE_START_TO_C;
    public static PathPlannerPath THREE_START_TO_D;
    public static PathPlannerPath THREE_START_TO_E;
    public static PathPlannerPath THREE_START_TO_F;
    public static PathPlannerPath THREE_START_TO_G;
    public static PathPlannerPath THREE_START_TO_H;
    public static PathPlannerPath THREE_START_TO_I;
    public static PathPlannerPath THREE_START_TO_J;
    public static PathPlannerPath THREE_START_TO_K;
    public static PathPlannerPath THREE_START_TO_L;

    // FOUR START
    public static PathPlannerPath FOUR_START_TO_A;
    public static PathPlannerPath FOUR_START_TO_B;
    public static PathPlannerPath FOUR_START_TO_C;
    public static PathPlannerPath FOUR_START_TO_D;
    public static PathPlannerPath FOUR_START_TO_E;
    public static PathPlannerPath FOUR_START_TO_F;
    public static PathPlannerPath FOUR_START_TO_G;
    public static PathPlannerPath FOUR_START_TO_H;
    public static PathPlannerPath FOUR_START_TO_I;
    public static PathPlannerPath FOUR_START_TO_J;
    public static PathPlannerPath FOUR_START_TO_K;
    public static PathPlannerPath FOUR_START_TO_L;

    // FIVE START
    public static PathPlannerPath FIVE_START_TO_A;
    public static PathPlannerPath FIVE_START_TO_B;
    public static PathPlannerPath FIVE_START_TO_C;
    public static PathPlannerPath FIVE_START_TO_D;
    public static PathPlannerPath FIVE_START_TO_E;
    public static PathPlannerPath FIVE_START_TO_F;
    public static PathPlannerPath FIVE_START_TO_G;
    public static PathPlannerPath FIVE_START_TO_H;
    public static PathPlannerPath FIVE_START_TO_I;
    public static PathPlannerPath FIVE_START_TO_J;
    public static PathPlannerPath FIVE_START_TO_K;
    public static PathPlannerPath FIVE_START_TO_L;

    // BRANCH TO ONE STATION
    public static PathPlannerPath A_TO_ONE_STATION;
    public static PathPlannerPath B_TO_ONE_STATION;
    public static PathPlannerPath C_TO_ONE_STATION;
    public static PathPlannerPath D_TO_ONE_STATION;
    public static PathPlannerPath E_TO_ONE_STATION;
    public static PathPlannerPath F_TO_ONE_STATION;
    public static PathPlannerPath G_TO_ONE_STATION;
    public static PathPlannerPath H_TO_ONE_STATION;
    public static PathPlannerPath I_TO_ONE_STATION;
    public static PathPlannerPath J_TO_ONE_STATION;
    public static PathPlannerPath K_TO_ONE_STATION;
    public static PathPlannerPath L_TO_ONE_STATION;

    // BRANCH TO TWO STATION
    public static PathPlannerPath A_TO_TWO_STATION;
    public static PathPlannerPath B_TO_TWO_STATION;
    public static PathPlannerPath C_TO_TWO_STATION;
    public static PathPlannerPath D_TO_TWO_STATION;
    public static PathPlannerPath E_TO_TWO_STATION;
    public static PathPlannerPath F_TO_TWO_STATION;
    public static PathPlannerPath G_TO_TWO_STATION;
    public static PathPlannerPath H_TO_TWO_STATION;
    public static PathPlannerPath I_TO_TWO_STATION;
    public static PathPlannerPath J_TO_TWO_STATION;
    public static PathPlannerPath K_TO_TWO_STATION;
    public static PathPlannerPath L_TO_TWO_STATION;

    // ONE STATION TO BRANCH
    public static PathPlannerPath ONE_STATION_TO_A;
    public static PathPlannerPath ONE_STATION_TO_B;
    public static PathPlannerPath ONE_STATION_TO_C;
    public static PathPlannerPath ONE_STATION_TO_D;
    public static PathPlannerPath ONE_STATION_TO_E;
    public static PathPlannerPath ONE_STATION_TO_F;
    public static PathPlannerPath ONE_STATION_TO_G;
    public static PathPlannerPath ONE_STATION_TO_H;
    public static PathPlannerPath ONE_STATION_TO_I;
    public static PathPlannerPath ONE_STATION_TO_J;
    public static PathPlannerPath ONE_STATION_TO_K;
    public static PathPlannerPath ONE_STATION_TO_L;

    // TWO STATION TO BRANCH
    public static PathPlannerPath TWO_STATION_TO_A;
    public static PathPlannerPath TWO_STATION_TO_B;
    public static PathPlannerPath TWO_STATION_TO_C;
    public static PathPlannerPath TWO_STATION_TO_D;
    public static PathPlannerPath TWO_STATION_TO_E;
    public static PathPlannerPath TWO_STATION_TO_F;
    public static PathPlannerPath TWO_STATION_TO_G;
    public static PathPlannerPath TWO_STATION_TO_H;
    public static PathPlannerPath TWO_STATION_TO_I;
    public static PathPlannerPath TWO_STATION_TO_J;
    public static PathPlannerPath TWO_STATION_TO_K;
    public static PathPlannerPath TWO_STATION_TO_L;

    static {
        try {
            ONE_START_TO_D = PathPlannerPath.fromPathFile("1,D");
            ONE_START_TO_E = PathPlannerPath.fromPathFile("1,E");

            TWO_START_TO_D = PathPlannerPath.fromPathFile("2,D");
            TWO_START_TO_E = PathPlannerPath.fromPathFile("2,E");

            THREE_START_TO_D = PathPlannerPath.fromPathFile("3,D");
            THREE_START_TO_E = PathPlannerPath.fromPathFile("3,E");

            FOUR_START_TO_A = PathPlannerPath.fromPathFile("4,A");
            FOUR_START_TO_B = PathPlannerPath.fromPathFile("4,B");
            FOUR_START_TO_C = PathPlannerPath.fromPathFile("4,C");
            FOUR_START_TO_D = PathPlannerPath.fromPathFile("4,D");
            FOUR_START_TO_E = PathPlannerPath.fromPathFile("4,E");
            FOUR_START_TO_F = PathPlannerPath.fromPathFile("4,F");
            FOUR_START_TO_G = PathPlannerPath.fromPathFile("4,G");
            FOUR_START_TO_H = PathPlannerPath.fromPathFile("4,H");
            FOUR_START_TO_I = PathPlannerPath.fromPathFile("4,I");
            FOUR_START_TO_J = PathPlannerPath.fromPathFile("4,J");
            FOUR_START_TO_K = PathPlannerPath.fromPathFile("4,K");
            FOUR_START_TO_L = PathPlannerPath.fromPathFile("4,L");

            FIVE_START_TO_B = PathPlannerPath.fromPathFile("5,B");
            FIVE_START_TO_D = PathPlannerPath.fromPathFile("5,D");
            FIVE_START_TO_E = PathPlannerPath.fromPathFile("5,E");

            A_TO_ONE_STATION = PathPlannerPath.fromPathFile("A,1-station");
            B_TO_ONE_STATION = PathPlannerPath.fromPathFile("B,1-station");
            C_TO_ONE_STATION = PathPlannerPath.fromPathFile("C,1-station");
            D_TO_ONE_STATION = PathPlannerPath.fromPathFile("D,1-station");
            E_TO_ONE_STATION = PathPlannerPath.fromPathFile("E,1-station");
            F_TO_ONE_STATION = PathPlannerPath.fromPathFile("F,1-station");
            
            D_TO_TWO_STATION = PathPlannerPath.fromPathFile("D,2-station");
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Failed to load paths: " + e.getMessage(), e.getStackTrace());
        }
    }

    public static AutoEventLooper generateAutoEventLooper(
            AutoChooser.StartPosition startPosition,
            List<AutoChooser.ScoringPositions> scoringLocations
    ) {
        return new AutoEventLooper("AutoEventLooper");
    }

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
        return PathManager.pathFindToClosePose(
                new Pose2d(
                        nearestAlgaeScoringPose.getTranslation(),
                        nearestAlgaeScoringPose.getRotation().rotateBy(Rotation2d.kPi)
                ),
                Constants.AutoConstants.DISTANCE_TOLERANCE_TO_DRIVE_INTO,
                1.0
        );
    }

    public static Command pathFindToNearestCoralScoringLocation(Pose2d currentPose) {
        Pose2d nearestCoralScoringPose = FieldUtil.Reef.getNearestBranchPose(currentPose);
        return PathManager.pathFindToClosePose(
                new Pose2d(
                        nearestCoralScoringPose.getTranslation(),
                        nearestCoralScoringPose.getRotation().rotateBy(Rotation2d.kPi)
                ),
                Constants.AutoConstants.DISTANCE_TOLERANCE_TO_DRIVE_INTO,
                1.0
        );
    }

    public static Command pathFindToClosePose(
            Pose2d targetPose,
            double distance,
            double goalEndVelocity
    ) {
        return pathFindToPose(
                new Pose2d(
                        targetPose.getTranslation().minus(new Translation2d(distance, targetPose.getRotation())),
                        targetPose.getRotation()
                ),
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

    private static boolean inBetween(Rotation2d angle, Rotation2d lowerAngleThreshold, Rotation2d upperAngleThreshold) {
        if (lowerAngleThreshold.getDegrees() > upperAngleThreshold.getDegrees()) {
            return angle.getDegrees() > lowerAngleThreshold.getDegrees() || angle.getDegrees() < upperAngleThreshold.getDegrees();
        }
        return angle.getDegrees() > lowerAngleThreshold.getDegrees() && angle.getDegrees() < upperAngleThreshold.getDegrees();
    }
}
