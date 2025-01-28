package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import org.json.simple.parser.ParseException;

import java.io.IOException;

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
    }

    public static Command pathFindToPose(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(
                targetPose,
                Constants.AutoConstants.PATH_CONSTRAINTS,
                0.0
        );
    }

    // TODO UPDATE THESE PRESET TARGET POSES (MEANT TO BE USED FOR SCORING OBJECTS)
    public static Command pathFindToNearestScoringLocation(Pose2d currentPose) {
        Translation2d currentTranslation = currentPose.getTranslation();
        ScoringLocations nearestLocation = ScoringLocations.STAGE;
        double nearestDistance = currentTranslation.getDistance(ScoringLocations.getScoringPose(nearestLocation).getTranslation());

        for (ScoringLocations location : ScoringLocations.values()) {
            double distance = currentTranslation.getDistance(ScoringLocations.getScoringPose(location).getTranslation());
            if (distance < nearestDistance) {
                nearestLocation = location;
                nearestDistance = distance;
            }
        }

        return pathFindToPose(ScoringLocations.getScoringPose(nearestLocation));
    }
}
