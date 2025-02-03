package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class PathManager {
    // TODO: CREATE 2025 PATHS
    public static PathPlannerPath ONE_START_TO_SIX_RIGHT;
    public static PathPlannerPath SIX_RIGHT_TO_STATION;
    public static PathPlannerPath STATION_TO_FIVE_LEFT;

    static {
        try {
            ONE_START_TO_SIX_RIGHT = PathPlannerPath.fromPathFile("1,6right");
            SIX_RIGHT_TO_STATION = PathPlannerPath.fromPathFile("6right,station");
            STATION_TO_FIVE_LEFT = PathPlannerPath.fromPathFile("station,5left");
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Failed to load paths: " + e.getMessage(), e.getStackTrace());
        }
    }

    // TODO: TEST & UPDATE SCORING LOCATIONS
    public enum ScoringLocations {
        AMP,
        STAGE,
        OPPONENT_SOURCE;

        // TODO IMPLEMENT RED SIDE POSES
        public static Pose2d getScoringPose(ScoringLocations location) {
            return switch (location) {
                case AMP -> new Pose2d(3.3, 6.35, Rotation2d.fromDegrees(-170));
                case STAGE -> new Pose2d(4.6, 4.85, Rotation2d.fromDegrees(166));
                case OPPONENT_SOURCE -> new Pose2d(2.7, 3.0, Rotation2d.fromDegrees(145));
            };
        }

        public static List<Pose2d> getScoringPoses() {
            List<Pose2d> poses = new ArrayList<>();
            Arrays.asList(ScoringLocations.values()).forEach(location -> poses.add(getScoringPose(location)));
            return poses;
        }
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

    // TODO UPDATE THESE PRESET TARGET POSES (MEANT TO BE USED FOR SCORING OBJECTS)
    public static Command pathFindToNearestScoringLocation(Pose2d currentPose) {
        return pathFindToPose(currentPose.nearest(ScoringLocations.getScoringPoses()));
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
