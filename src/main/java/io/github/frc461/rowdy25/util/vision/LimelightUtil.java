package io.github.frc461.rowdy25.util.vision;

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import io.github.frc461.rowdy25.constants.Constants;

/**
 * Utility class for interfacing with the Limelight camera, henceforth called Limelight. This class provides methods to retrieve and process data from the Limelight NetworkTable (e.g., target pose, latency, tag information), as well as various configuration and calibration methods.
 *
 * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api">NetworkTables API</a> for information on how to use the Limelight NetworkTable to retrieve/update desired values. Clarify class methods/string key entries in this documentation.
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public final class LimelightUtil {
    /** The Limelight network table. */
    private static final NetworkTable LIMELIGHT_NT = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.LimelightConstants.LIMELIGHT_NT_NAME);

    /**
     * Retrieves and returns the Limelight NetworkTable entry that contains data relating to the pose of the "primary in-view" AprilTag (target) relative to the robot.
     *
     * @return A double array containing the x, y, z, roll, pitch, and yaw components representing the pose of the detected AprilTag (target) relative to the robot. If the entry is empty (if no AprilTag is detected), an empty array is returned.
     */
    private static double[] getTargetPoseRobotSpaceValues() {
        return LIMELIGHT_NT.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
    }

    /**
     * Retrieves and returns the Limelight NetworkTable entry that contains data relating to the robot pose using the detected AprilTag(s) using the MegaTag1 protocol.
     *
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization">Limelight MegaTag1 Documentation</a
     * @return A double array containing the x, y, z, roll, pitch, and yaw components (among other values) representing the localized robot pose using MegaTag1. If the entry is empty (if no AprilTag is detected), an empty array is returned.
     */
    private static double[] getMegaTagOneValues() {
        return LIMELIGHT_NT.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    }

    /**
     * Retrieves and returns the Limelight NetworkTable entry that contains data relating to the robot pose using the detected AprilTag(s) using the MegaTag2 protocol.
     *
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2">Limelight MegaTag2 Documentation</a>
     * @return A double array containing the x, y, z, roll, pitch, and yaw components (among other values) representing the localized robot pose using MegaTag2. If the entry is empty (if no AprilTag is detected), an empty array is returned.
     */
    private static double[] getMegaTagTwoValues() {
        return LIMELIGHT_NT.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
    }

    /**
     * Retrieves and returns the Limelight NetworkTable entry that contains data relating to the latency of the Limelight's processing pipeline and the capture latency of the camera. The total latency is calculated by summing the pipeline latency ("tl") and the capture latency ("cl"), and converting the result from milliseconds to seconds.
     *
     * @return A double representing the total latency of the Limelight's processing pipeline and camera capture, in seconds. If the entry is empty, a default value of 0.0 seconds is returned.
     */
    public static double getLatency() {
        return (LIMELIGHT_NT.getEntry("tl").getDouble(0.0) + LIMELIGHT_NT.getEntry("cl").getDouble(0.0)) / 1000.0;
    }

    /**
     * Retrieves and returns the Limelight NetworkTable entry that contains data relating to the fiducial ID of the "primary in-view" AprilTag.
     *
     * @return A double representing the fiducial ID of the "primary in-view" AprilTag. If the entry is empty (if no AprilTag is detected), a default value of 0.0 is returned.
     */
    public static double getBestTagID() {
        return LIMELIGHT_NT.getEntry("tid").getDouble(0.0);
    }

    /**
     * Retrieves and returns the Limelight NetworkTable entry that contains data relating to whether any AprilTags (targets) are detected.
     *
     * @return A boolean representing whether any AprilTags are currently detected.
     */
    public static boolean tagExists() {
        return LIMELIGHT_NT.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Retrieves and returns the number of detected AprilTags. This is obtained using the {@link #getMegaTagOneValues()} method, as the eighth entry in the returned array represents the number of tags detected by the Limelight. If the MegaTag1 values array is empty or does not contain at least 8 entries, a default value of 0 is returned.
     *
     * @return An integer representing the number of detected AprilTags.
     */
    public static int getNumTags() {
        double[] values = getMegaTagOneValues();

        // The 8th entry in the MegaTag network table is the number of tags the camera detects
        if (values.length < 8) {
            return 0;
        }
        return (int) values[7];
    }

    /**
     * Parses the target-pose-robot-space array and returns the representative {@link Pose2d}. If entry is empty, the default {@link Pose2d} is returned.
     *
     * @see <a href="https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems">Limelight 3D Coordinate Systems In Detail</a> to see why which values are parsed/included in the final pose.
     * @return A {@link Pose2d} representing the pose of the "primary in-view" AprilTag relative to the robot.
     */
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

    /**
     * Parses the MegaTag1 array and returns the representative {@link Pose2d}. If entry is empty, the default {@link Pose2d} is returned. Note the @see tag in the {@link #getTargetPoseRobotSpace()} method documentation.
     *
     * @return A {@link Pose2d} representing the localized robot pose using MegaTag1.
     */
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


    /**
     * Parses the MegaTag2 array and returns the representative {@link Pose2d}. If entry is empty, the default {@link Pose2d} is returned. Note the @see tag in the {@link #getTargetPoseRobotSpace()} method documentation.
     *
     * @return A {@link Pose2d} representing the localized robot pose using MegaTag2.
     */
    public static Pose2d getMegaTagTwoPose() {
        double[] values = getMegaTagTwoValues();

        if (values.length < 6) {
            return new Pose2d();
        }
        return new Pose2d(
                new Translation2d(values[0], values[1]),
                new Rotation2d(Units.degreesToRadians(values[5]))
        );
    }

    /**
     * Uses the "primary in-view" AprilTag pose relative to the robot to obtain the distance (in meters) from the robot to the "primary in-view" AprilTag, henceforth called the "nearest tag" (which may be inaccurate). If entry is empty, 0.0 (from the default {@link Pose2d}) is returned.
     *
     * @return The distance (in meters) of the nearest tag from the robot.
     */
    public static double getNearestTagDist() {
        return getTargetPoseRobotSpace().getTranslation().getNorm();
    }

    /**
     * Checks whether the Limelight detects at least 2 AprilTags
     *
     * @return A boolean representing whether the Limelight detects at least 2 AprilTags.
     */
    public static boolean isMultiTag() {
        return getNumTags() >= 2;
    }

    /**
     * Checks whether an AprilTag is currently detected and if the nearest tag is within the maximum clear distance threshold.
     *
     * @return A boolean indicating whether a detected AprilTag is considered clear and reliable.
     */
    public static boolean isTagClear() {
        return tagExists() && getNearestTagDist() < Constants.VisionConstants.LimelightConstants.LL_MAX_TAG_CLEAR_DIST;
    }

    /**
     * Configures the camera's pose relative to the robot (perspective-relative offset) by updating the Limelight NetworkTable with the specified forward, right, up, roll, pitch, and yaw constants.
     */
    public static void configureRobotToCameraOffset() {
        LIMELIGHT_NT.getEntry("camerapose_robotspace_set").setDoubleArray(
                new double[]{
                        Constants.VisionConstants.LimelightConstants.LL_FORWARD,
                        Constants.VisionConstants.LimelightConstants.LL_RIGHT,
                        Constants.VisionConstants.LimelightConstants.LL_UP,
                        Constants.VisionConstants.LimelightConstants.LL_ROLL,
                        Constants.VisionConstants.LimelightConstants.LL_PITCH,
                        Constants.VisionConstants.LimelightConstants.LL_YAW
                }
        );
    }

    /**
     * Calibrates the robot's yaw orientation to enhance MegaTag2 localization data by updating the Limelight NetworkTables with the specified yaw.
     *
     * @param yaw The current yaw of the robot in degrees.
     */
    public static void calibrateRobotOrientation(double yaw) {
        LIMELIGHT_NT.getEntry("robot_orientation_set").setDoubleArray(
                new double[]{yaw, 0, 0, 0, 0, 0}
        );
    }
}
