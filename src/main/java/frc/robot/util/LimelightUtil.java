package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;

public class LimelightUtil {
    private static final NetworkTable LIMELIGHT_NT = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.LIMELIGHT_NT_NAME);

    private static double[] getTargetPoseRobotSpaceValues() {
        return LIMELIGHT_NT.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
    }

    private static double[] getMegaTagOneValues() {
        return LIMELIGHT_NT.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    }

    // Get pipeline latency + capture latency
    public static double getLatency() {
        return (LIMELIGHT_NT.getEntry("tl").getDouble(0.0) + LIMELIGHT_NT.getEntry("cl").getDouble(0.0)) / 1000.0;
    }

    public static double getPrimaryFiducialID() {
        return LIMELIGHT_NT.getEntry("tid").getDouble(0.0);
    }

    public static boolean tagExists() {
        return LIMELIGHT_NT.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public static int getNumTags() {
        double[] values = getMegaTagOneValues();

        // The 8th entry in the MegaTag network table is the number of tags the camera detects
        if (values.length < 8) {
            return 0;
        }
        return (int) values[7];
    }

    public static Pose2d getTargetPoseRobotSpace() {
        double[] values = getTargetPoseRobotSpaceValues();

        if (values.length < 6) {
            return new Pose2d();
        }
        return new Pose2d(
                new Translation2d(values[0], values[2]),
                new Rotation2d(Units.degreesToRadians(values[5]))
        );
    }

    public static Pose2d getMegaTagOnePose() {
        double[] values = getMegaTagOneValues();

        if (values.length < 6) {
            return new Pose2d();
        }
        return new Pose2d(
                new Translation2d(values[0], values[1]),
                new Rotation2d(Units.degreesToRadians(values[5]))
        );
    }

    public static double getNearestTagDist() {
        return getTargetPoseRobotSpace().getTranslation().getNorm();
    }

    public static void configureCameraPose() {
        LIMELIGHT_NT.getEntry("camerapose_robotspace_set").setDoubleArray(
                new double[] {
                        Constants.VisionConstants.CAMERA_FORWARD,
                        0,
                        Constants.VisionConstants.CAMERA_UP,
                        0,
                        Constants.VisionConstants.CAMERA_PITCH,
                        0
                }
        );
    }

    public static void calibrateRobotOrientation(double yaw) {
        LIMELIGHT_NT.getEntry("robot_orientation_set").setDoubleArray(
                new double[] {yaw, 0, 0, 0, 0, 0}
        );
    }
}
