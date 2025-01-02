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

public final class PathManager {
    public static PathPlannerPath TEST_PATH;
    public static PathPlannerPath TEST_PATH_2;
    public static PathPlannerPath TEST_PATH_3;

    // TODO ENUM FOR DIFFERENT SHOOTING PLACES

    static {
        try {
            TEST_PATH = PathPlannerPath.fromPathFile("Test");
            TEST_PATH_2 = PathPlannerPath.fromPathFile("Test2");
            TEST_PATH_3 = PathPlannerPath.fromPathFile("Test3");
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Failed to load paths: " + e.getMessage(), e.getStackTrace());
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
    public static Command pathFindToAmpSide() {
        return pathFindToPose(new Pose2d(3.5, 7.0, new Rotation2d()));
    }
}
