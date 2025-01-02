package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class PathFinder {
    private static Command pathFindToPose(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                Constants.MAX_VEL,
                Constants.MAX_ACCEL,
                Constants.MAX_DESIRED_ANGULAR_VEL,
                Constants.MAX_ANGULAR_ACCEL
        );

        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0
        );
    }

    // TODO UPDATE THESE PRESET TARGET POSES (MEANT TO BE USED FOR SCORING, NOT PICKING UP, WHICH IS SUPPOSED TO BE COMPLETELY DYNAMIC)
    public static Command pathFindToAmpSide() {
        return pathFindToPose(new Pose2d(3.5, 7.0, new Rotation2d()));
    }
}
