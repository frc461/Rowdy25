package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

public class VisionUtil {
    public static boolean highConfidenceEstimation() {
        return Limelight.isTagClear() && Photon.BW.isTagClear();
    }

    public static final class Limelight {
        private static final NetworkTable LIMELIGHT_NT = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.LimelightConstants.LIMELIGHT_NT_NAME);

        private static double[] getTargetPoseRobotSpaceValues() {
            return LIMELIGHT_NT.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
        }

        private static double[] getMegaTagOneValues() {
            return LIMELIGHT_NT.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        }

        private static double[] getMegaTagTwoValues() {
            return LIMELIGHT_NT.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
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

        public static double getNearestTagDist() {
            return getTargetPoseRobotSpace().getTranslation().getNorm();
        }

        public static boolean isMultiTag() {
            return getNumTags() >= 2;
        }

        public static boolean isTagClear() {
            return tagExists() && getNearestTagDist() < Constants.VisionConstants.LimelightConstants.LL_MAX_TAG_CLEAR_DIST;
        }

        public static void configureRobotToCameraOffset() {
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
        public static void updateResults() {
            Color.updateResults();
            BW.updateResults();
        }

        public static final class Color {
            private static final PhotonCamera COLOR = new PhotonCamera(Constants.NT_INSTANCE, "ArducamColor");
            private static PhotonPipelineResult latestResult = new PhotonPipelineResult();

            public static boolean hasTargets() {
                return latestResult.hasTargets();
            }

            public static double getBestObjectYaw() {
                return hasTargets() ? latestResult.getBestTarget().getYaw() : 0.0;
            }

            public static double getBestObjectPitch() {
                return hasTargets() ? latestResult.getBestTarget().getPitch() : 0.0;
            }

            public static void updateResults() {
                List<PhotonPipelineResult> results = COLOR.getAllUnreadResults();
                if (!results.isEmpty()) {
                    latestResult = results.get(results.size() - 1);
                }
            }
        }

        public static final class BW {
            private static final PhotonCamera BW = new PhotonCamera(Constants.NT_INSTANCE, "ArducamBW");
            public static final AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // TODO UPDATE FOR 2025
            public static final Transform3d robotToCameraOffset = new Transform3d(
                    Constants.VisionConstants.PhotonConstants.BW_FORWARD,
                    Constants.VisionConstants.PhotonConstants.BW_LEFT,
                    Constants.VisionConstants.PhotonConstants.BW_UP,
                    new Rotation3d(
                            Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_ROLL),
                            Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_PITCH),
                            Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_YAW)
                    )
            );

            // Photon Vision's integrated estimator, to be integrated into the localizer's pose estimator
            private static final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
                    tagLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCameraOffset
            );

            private static Optional<EstimatedRobotPose> poseEstimateOpt = Optional.empty();

            private static PhotonPipelineResult latestResult = new PhotonPipelineResult();

            public static boolean hasTargets() {
                return latestResult.hasTargets();
            }

            public static double getLatestResultTimestamp() {
                return latestResult.getTimestampSeconds();
            }

            public static double getBestTagID() {
                return hasTargets() ? latestResult.getBestTarget().fiducialId : 0.0;
            }

            public static double getBestTagDist() {
                return hasTargets()
                        ? latestResult.getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d().getNorm()
                        : 0.0;
            }

            public static boolean isMultiTag() {
                return latestResult.targets.size() >= 2;
            }

            public static boolean isTagClear() {
                return hasTargets() && getBestTagDist() < Constants.VisionConstants.PhotonConstants.BW_MAX_TAG_CLEAR_DIST;
            }

            public static Optional<EstimatedRobotPose> getOptionalPoseData() {
                return poseEstimateOpt;
            }

            public static Pose2d getMultiTagPose() {
                Optional<EstimatedRobotPose> pose = getOptionalPoseData();
                return pose.isPresent() ? pose.get().estimatedPose.toPose2d() : new Pose2d();
            }

            // TODO TEST THIS
            public static Pose2d getSingleTagPose(Pose2d currentPose) {
                if (hasTargets()) {

                    Pose3d tagPose = FieldUtil.TagLocation.getTagLocation3d(getBestTagID());
                    Transform3d cameraToTargetBest = latestResult.getBestTarget().getBestCameraToTarget();
                    Transform3d cameraToTargetAlt = latestResult.getBestTarget().getAlternateCameraToTarget();

                    Pose2d poseBest = tagPose.plus(cameraToTargetBest.inverse()).plus(robotToCameraOffset.inverse()).toPose2d();
                    Pose2d poseAlt = tagPose.plus(cameraToTargetAlt.inverse()).plus(robotToCameraOffset.inverse()).toPose2d();

                    Pose2d poseToReturn;

                    // Disambiguate using the current pose i.e., choose the pose most consistent with the robot's current rotation
                    if (latestResult.getBestTarget().getPoseAmbiguity() < 0.15) {
                        poseToReturn = poseBest;
                    } else if (Math.abs(poseBest.getRotation().minus(currentPose.getRotation()).getDegrees()) <
                            Math.abs(poseAlt.getRotation().minus(currentPose.getRotation()).getDegrees())) {
                        poseToReturn = poseBest;
                    } else {
                        poseToReturn = poseAlt;
                    }

                    // Check if the pose is inside the field
                    if (poseToReturn.getX() > 0 && poseToReturn.getY() > 0
                            && poseToReturn.getX() < FieldUtil.FIELD_LENGTH && poseToReturn.getY() < FieldUtil.FIELD_WIDTH) {
                        return poseToReturn;
                    }
                }
                return new Pose2d();
            }

            public static void updateResults() {
                List<PhotonPipelineResult> results = BW.getAllUnreadResults();
                if (!results.isEmpty()) {
                    latestResult = results.get(results.size() - 1);
                    poseEstimateOpt = photonPoseEstimator.update(Photon.BW.latestResult);
                }
            }
        }
    }

    public static final class QuestNav {
        private static final NetworkTable QUESTNAV_NT = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.QuestNavConstants.QUESTNAV_NT_NAME);

        private static final IntegerSubscriber questMiso = QUESTNAV_NT.getIntegerTopic("miso").subscribe(0);
        private static final IntegerPublisher questMosi = QUESTNAV_NT.getIntegerTopic("mosi").publish();

        private static final FloatArraySubscriber questPositionTopic = QUESTNAV_NT.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
        private static final FloatArraySubscriber questEulerAnglesTopic = QUESTNAV_NT.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});

        // Transformation applied to QuestNav pose to adjust origin to the pose estimator's origin
        public static final Transform2d robotToCameraOffset = new Transform2d(
                new Translation2d(
                        Constants.VisionConstants.QuestNavConstants.QUEST_FORWARD,
                        Constants.VisionConstants.QuestNavConstants.QUEST_LEFT
                ),
                new Rotation2d(Units.degreesToRadians(Constants.VisionConstants.QuestNavConstants.QUEST_YAW))
        );

        public static Transform2d questToFieldOffset = new Transform2d();

        public static double getRawX() {
            return questPositionTopic.get()[2];
        }

        public static double getRawY() {
            return -questPositionTopic.get()[0];
        }

        public static double getRawZ() {
            return questPositionTopic.get()[1];
        }

        public static double stabilize(double angle) {
            return angle >= 180
                    ? angle - ((int) ((angle - 180) / 360)) * 360 - 360
                    : angle <= -180
                            ? angle - ((int) ((angle + 180) / 360)) * 360 + 360
                            : angle;
        }

        public static double getRawPitch() {
            return stabilize(questEulerAnglesTopic.get()[0]);
        }

        public static double getRawYaw() {
            return stabilize(-questEulerAnglesTopic.get()[1]);
        }

        public static double getRawRoll() {
            return stabilize(questEulerAnglesTopic.get()[2]);
        }

        public static Pose2d getRawPose() {
            return new Pose2d(
                    new Translation2d(getRawX(), getRawY()),
                    new Rotation2d(Units.degreesToRadians(getRawYaw()))
            );
        }

        public static Pose2d getFinalCameraPose() {
            Pose2d rawPose = getRawPose();
            Translation2d correctedTranslation = rawPose.getTranslation().rotateBy(questToFieldOffset.getRotation());
            return new Pose2d(
                    questToFieldOffset.getTranslation().plus(correctedTranslation),
                    questToFieldOffset.getRotation().rotateBy(rawPose.getRotation())
            );
        }

        public static Pose2d getFinalRobotPose() {
            return getFinalCameraPose().plus(robotToCameraOffset.inverse());
        }

        public static void zeroQuestPose() {
            if (questMiso.get() != 99) {
                questMosi.set(1);
            }
        }

        public static void completeQuestPose() {
            if (questMiso.get() == 99) {
                questMosi.set(0);
            }
        }

        public static void setQuestPose(Pose2d pose) {
            zeroQuestPose();
            Transform2d correctedRobotToCameraOffset = new Transform2d(
                    robotToCameraOffset.getTranslation().rotateBy(pose.getRotation()),
                    robotToCameraOffset.getRotation()
            );
            questToFieldOffset = new Transform2d(
                    pose.getTranslation().plus(correctedRobotToCameraOffset.getTranslation()),
                    pose.getRotation().rotateBy(correctedRobotToCameraOffset.getRotation())
            );
        }
    }
}
