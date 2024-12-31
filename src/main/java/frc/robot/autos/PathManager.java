package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public final class PathManager {
    public static PathPlannerPath TEST_PATH;

    static {
        try {
            TEST_PATH = PathPlannerPath.fromPathFile("Test");
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Failed to load paths: " + e.getMessage(), e.getStackTrace());
        }
    }
}
