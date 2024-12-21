package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;

public class VisionUtil {
    private static final NetworkTable LIMELIGHT_NT = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.LIMELIGHT_NT_NAME);
    private static final NetworkTable OCULUS_NT = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.OCULUS_NT_NAME);

    // TODO USE FOR ADSCOPE
    private static final IntegerSubscriber questFrameCountTopic = OCULUS_NT.getIntegerTopic("frameCount").subscribe(0);
    private static final DoubleSubscriber questTimestampTopic = OCULUS_NT.getDoubleTopic("timestamp").subscribe(0.0f);
    private static final DoubleSubscriber questBatteryTopic = OCULUS_NT.getDoubleTopic("batteryLevel").subscribe(0.0f);
    private static final FloatArraySubscriber questPositionTopic = OCULUS_NT.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private static final FloatArraySubscriber questQuaternionTopic = OCULUS_NT.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    private static final FloatArraySubscriber questEulerAnglesTopic = OCULUS_NT.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});

    public static final class Limelight {
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

        public static boolean isTagClear() {
            return tagExists() && getNearestTagDist() < Constants.VisionConstants.MIN_TAG_CLEAR_DIST;
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

    public static final class Oculus {
        public static Pose2d offset = new Pose2d();

        public static double getX() {
            return questPositionTopic.get()[2];
        }

        public static double getY() {
            return -questPositionTopic.get()[0];
        }

        public static double getZ() {
            return questPositionTopic.get()[2];
        }

        public static double getPitch() {
            return questEulerAnglesTopic.get()[0];
        }

        public static double getYaw() {
            return questEulerAnglesTopic.get()[1];
        }

        public static double getRoll() {
            return questEulerAnglesTopic.get()[2];
        }

        public static double getTimestamp() {
            return questTimestampTopic.get();
        }

        public static Pose2d getPose() {
            return new Pose2d(
                    new Translation2d(
                            getX(),
                            getY()
                    ),
                    new Rotation2d(getYaw())
            );
        }

        public static void setOffset() {
            if (Limelight.isTagClear()) {
                offset = Limelight.getMegaTagOnePose();
            }
        }
    }
}
