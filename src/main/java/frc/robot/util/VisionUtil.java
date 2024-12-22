package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class VisionUtil {

    public static final class Limelight {
        private static final NetworkTable LIMELIGHT_NT = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.LimelightConstants.LIMELIGHT_NT_NAME);

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

        public static double getBestTagID() {
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
            return tagExists() && getNearestTagDist() < Constants.VisionConstants.LimelightConstants.LL_MAX_TAG_CLEAR_DIST;
        }

        public static void configureRobotToCameraTransform() {
            LIMELIGHT_NT.getEntry("camerapose_robotspace_set").setDoubleArray(
                    new double[] {
                            Constants.VisionConstants.LimelightConstants.LL_FORWARD,
                            Constants.VisionConstants.LimelightConstants.LL_RIGHT,
                            Constants.VisionConstants.LimelightConstants.LL_UP,
                            Constants.VisionConstants.LimelightConstants.LL_ROLL,
                            Constants.VisionConstants.LimelightConstants.LL_PITCH,
                            Constants.VisionConstants.LimelightConstants.LL_YAW
                    }
            );
        }

        public static void calibrateRobotOrientation(double yaw) {
            LIMELIGHT_NT.getEntry("robot_orientation_set").setDoubleArray(
                    new double[] {yaw, 0, 0, 0, 0, 0}
            );
        }
    }

    public static final class Photon {

        public static final class Color {
            private static final PhotonCamera COLOR = new PhotonCamera(Constants.NT_INSTANCE, "ArducamColor");
            private static List<PhotonPipelineResult> results;

            public static boolean hasResults() {
                return !results.isEmpty();
            }

            public static boolean hasTargets() {
                return hasResults() && results.get(results.size() - 1).hasTargets();
            }

            public static PhotonTrackedTarget getBestObject() {
                return results.get(results.size() - 1).getBestTarget();
            }

            public static double getBestObjectYaw() {
                return hasTargets() ? getBestObject().getYaw() : 0.0;
            }

            // TODO UPDATE ONCE EVERY TICK
            public static void updateColorResults() {
                results = COLOR.getAllUnreadResults();
            }
        }

        public static final class BW {
            private static final PhotonCamera BW = new PhotonCamera(Constants.NT_INSTANCE, "ArducamBW");
            public static final AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // TODO UPDATE FOR 2025
            public static final Transform3d cameraTransform = new Transform3d(
                    Constants.VisionConstants.PhotonConstants.BW_FORWARD,
                    Constants.VisionConstants.PhotonConstants.BW_LEFT,
                    Constants.VisionConstants.PhotonConstants.BW_UP,
                    new Rotation3d(
                            Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_ROLL),
                            Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_PITCH),
                            Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_YAW)
                    )
            );
            public static List<PhotonPipelineResult> results;

            public static boolean hasResults() {
                return !results.isEmpty();
            }

            public static boolean hasTargets() {
                return hasResults() && results.get(results.size() - 1).hasTargets();
            }

            public static PhotonTrackedTarget getBestTag() {
                return results.get(results.size() - 1).getBestTarget();
            }

            public static double getLatency() {
                return hasResults() ? results.get(results.size() - 1).getTimestampSeconds() : 0.0;
            }

            public static double getBestTagID() {
                return hasTargets() ? getBestTag().fiducialId : 0.0;
            }

            public static double getBestTagDist() {
                return hasTargets()
                        ? getBestTag().getBestCameraToTarget().getTranslation().toTranslation2d().getNorm()
                        : 0.0;
            }

            public static boolean isTagClear() {
                return hasTargets() && getBestTagDist() < Constants.VisionConstants.PhotonConstants.BW_MAX_TAG_CLEAR_DIST;
            }

            // TODO TEST THIS AFTER MULTITAG IF MULTITAG DOESN'T WORK
            public static Pose2d getPhotonPose() {
                return PhotonUtils.estimateFieldToRobotAprilTag(
                        getBestTag().getBestCameraToTarget(),
                        TagLocation.getTagLocation3d(getBestTagID()),
                        cameraTransform
                ).toPose2d();
            }

            // TODO UPDATE ONCE EVERY TICK
            public static void updateBWResults() {
                results = BW.getAllUnreadResults();
            }
        }
    }

    public static final class QuestNav {
        private static final NetworkTable QUESTNAV_NT = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.QuestNavConstants.QUESTNAV_NT_NAME);

        private static final DoubleSubscriber questTimestampTopic = QUESTNAV_NT.getDoubleTopic("timestamp").subscribe(0.0f);
        private static final FloatArraySubscriber questPositionTopic = QUESTNAV_NT.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
        private static final FloatArraySubscriber questEulerAnglesTopic = QUESTNAV_NT.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});

        public static Transform2d poseEstimateOffset = new Transform2d();
        public static Transform2d diffMegaTagOneQuest = new Transform2d();

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
            ).plus(poseEstimateOffset);
        }

        // TODO SET OFFSET WITH POSE ESTIMATE AS REFERENCE INSTEAD OF LIMELIGHT MEGATAG
        public static void setOffset() {
            if (Limelight.isTagClear()) {
                poseEstimateOffset = new Transform2d(
                        Limelight.getMegaTagOnePose().getX(),
                        Limelight.getMegaTagOnePose().getY(),
                        Limelight.getMegaTagOnePose().getRotation()
                );
            }
        }

        public static void updateOffset() {
            if (Limelight.isTagClear()) {
                diffMegaTagOneQuest = Limelight.getMegaTagOnePose().minus(getPose());
                double dist = diffMegaTagOneQuest.getTranslation().getNorm();
                double rot = diffMegaTagOneQuest.getRotation().getDegrees();
                if (dist > Constants.VisionConstants.UPDATE_QUEST_OFFSET_TRANSLATION_ERROR_THRESHOLD
                        || rot > Constants.VisionConstants.UPDATE_QUEST_OFFSET_ROTATION_ERROR_THRESHOLD) {
                    poseEstimateOffset = poseEstimateOffset.plus(diffMegaTagOneQuest);
                }
            }
        }

        public static void setPose(Pose2d pose) {
            poseEstimateOffset = poseEstimateOffset.plus(pose.minus(getPose()));
        }
    }
}
