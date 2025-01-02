package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.Supplier;

public final class PathManager {
    public static PathPlannerPath TEST_PATH;
    public static PathPlannerPath TEST_PATH_2;
    public static PathPlannerPath TEST_PATH_3;

    static {
        try {
            TEST_PATH = PathPlannerPath.fromPathFile("Test");
            TEST_PATH_2 = PathPlannerPath.fromPathFile("Test2");
            TEST_PATH_3 = PathPlannerPath.fromPathFile("Test3");
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Failed to load paths: " + e.getMessage(), e.getStackTrace());
        }
    }

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

    private static Command pathFindToPose(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(
                targetPose,
                Constants.AutoConstants.PATH_CONSTRAINTS,
                0.0
        );
    }

    // TODO UPDATE THESE PRESET TARGET POSES (MEANT TO BE USED FOR SCORING, NOT PICKING UP, WHICH IS SUPPOSED TO BE COMPLETELY DYNAMIC)
    public static Command pathFindToNearestShootingLocation(Supplier<Pose2d> poseSupplier) {
        Pose2d current = poseSupplier.get();
        ScoringLocations nearestLocation = ScoringLocations.STAGE;
        double nearestDistance = current.getTranslation().getDistance(ScoringLocations.getScoringPose(nearestLocation).getTranslation());

        for (ScoringLocations location : ScoringLocations.values()) {
            double distance = current.getTranslation().getDistance(ScoringLocations.getScoringPose(location).getTranslation());
            if (distance < nearestDistance) {
                nearestLocation = location;
                nearestDistance = distance;
            }
        }

        return pathFindToPose(ScoringLocations.getScoringPose(nearestLocation));
    }
}
