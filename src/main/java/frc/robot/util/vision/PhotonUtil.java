package frc.robot.util.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.util.EstimatedRobotPose;
import frc.robot.util.FieldUtil;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public final class PhotonUtil {
    public static void updateResults() {
        Color.updateResults();
        BW.updateResults();
    }

    public static final class Color {
        public enum TargetClass {
            ALGAE(0),
            CORAL(1),
            NONE(-1);

            public final int id;

            TargetClass(int id) {
                this.id = id;
            }

            public static Color.TargetClass fromID(int id) {
                return switch (id) {
                    case 0 -> ALGAE;
                    case 1 -> CORAL;
                    default -> NONE;
                };
            }
        }

        private static final PhotonCamera COLOR = new PhotonCamera(Constants.NT_INSTANCE, "ArducamColor");
        private static PhotonPipelineResult latestResult = new PhotonPipelineResult();

        public static boolean hasTargets() {
            return latestResult.hasTargets();
        }

        public static boolean hasTargets(Color.TargetClass targetClass) {
            if (hasTargets()) {
                for (PhotonTrackedTarget target : latestResult.getTargets()) {
                    if (target.getDetectedObjectClassID() == targetClass.id) {
                        return true;
                    }
                }
            }
            return false;
        }

        // TODO WAIT (COLOR CAMERA): TEST ALGAE AND CORAL SPECIFIC TARGETING
        public static boolean hasAlgaeTargets() {
            return hasTargets(Color.TargetClass.ALGAE);
        }

        public static boolean hasCoralTargets() {
            return hasTargets(Color.TargetClass.CORAL);
        }

        public static Color.TargetClass getBestObjectClass() {
            return hasTargets() ? Color.TargetClass.fromID(latestResult.getBestTarget().getDetectedObjectClassID()) : Color.TargetClass.NONE;
        }

        public static double getBestObjectYaw() {
            return hasTargets() ? latestResult.getBestTarget().getYaw() : 0.0;
        }

        public static double getBestObjectPitch() {
            return hasTargets() ? latestResult.getBestTarget().getPitch() : 0.0;
        }

        public static double getBestObjectYaw(Color.TargetClass targetClass) {
            if (hasTargets(targetClass)) {
                for (PhotonTrackedTarget target : latestResult.getTargets()) {
                    if (target.getDetectedObjectClassID() == targetClass.id) {
                        return target.getYaw();
                    }
                }
            }
            return 0.0;
        }

        public static double getBestObjectPitch(Color.TargetClass targetClass) {
            if (hasTargets(targetClass)) {
                for (PhotonTrackedTarget target : latestResult.getTargets()) {
                    if (target.getDetectedObjectClassID() == targetClass.id) {
                        return target.getPitch();
                    }
                }
            }
            return 0.0;
        }

        public static void updateResults() {
            List<PhotonPipelineResult> results = COLOR.getAllUnreadResults();
            if (!results.isEmpty()) {
                latestResult = results.get(results.size() - 1);
            }
        }
    }

    public static final class BW {
        public enum BWCamera {
            TOP_RIGHT,
            TOP_LEFT,
            BACK
        }

        private static final PhotonCamera BW_TOP_RIGHT = new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_NAME);
        private static final PhotonCamera BW_TOP_LEFT = new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_NAME);
        private static final PhotonCamera BW_BACK = new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.PhotonConstants.BW_BACK_NAME);

        private static PhotonPipelineResult latestResultTopRight = new PhotonPipelineResult();
        private static PhotonPipelineResult latestResultTopLeft = new PhotonPipelineResult();
        private static PhotonPipelineResult latestResultBack = new PhotonPipelineResult();

        private static final Transform3d ROBOT_TO_BW_TOP_RIGHT_OFFSET = new Transform3d(
                Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_FORWARD,
                Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_LEFT,
                Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_UP,
                new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_YAW)
                )
        );

        private static final Transform3d ROBOT_TO_BW_TOP_LEFT_OFFSET = new Transform3d(
                Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_FORWARD,
                Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_LEFT,
                Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_UP,
                new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_YAW)
                )
        );

        private static final Transform3d ROBOT_TO_BW_BACK_OFFSET = new Transform3d(
                Constants.VisionConstants.PhotonConstants.BW_BACK_FORWARD,
                Constants.VisionConstants.PhotonConstants.BW_BACK_LEFT,
                Constants.VisionConstants.PhotonConstants.BW_BACK_UP,
                new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_BACK_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_BACK_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_BACK_YAW)
                )
        );

        public static PhotonCamera getCamera(BW.BWCamera camera) {
            return switch (camera) {
                case TOP_RIGHT -> BW_TOP_RIGHT;
                case TOP_LEFT -> BW_TOP_LEFT;
                case BACK -> BW_BACK;
            };
        }

        public static PhotonPipelineResult getLatestResult(BW.BWCamera camera) {
            return switch (camera) {
                case TOP_RIGHT -> latestResultTopRight;
                case TOP_LEFT -> latestResultTopLeft;
                case BACK -> latestResultBack;
            };
        }

        public static Transform3d getRobotToBWOffset(BW.BWCamera camera) {
            return switch (camera) {
                case TOP_RIGHT -> ROBOT_TO_BW_TOP_RIGHT_OFFSET;
                case TOP_LEFT -> ROBOT_TO_BW_TOP_LEFT_OFFSET;
                case BACK -> ROBOT_TO_BW_BACK_OFFSET;
            };
        }

        public static boolean hasTargets(BW.BWCamera camera) {
            return switch (camera) {
                case TOP_RIGHT -> latestResultTopRight.hasTargets();
                case TOP_LEFT -> latestResultTopLeft.hasTargets();
                case BACK -> latestResultBack.hasTargets();
            };
        }

        public static double getLatestResultTimestamp(BW.BWCamera camera) {
            return switch (camera) {
                case TOP_RIGHT -> latestResultTopRight.getTimestampSeconds();
                case TOP_LEFT -> latestResultTopLeft.getTimestampSeconds();
                case BACK -> latestResultBack.getTimestampSeconds();
            };
        }

        public static double getBestTagID(BW.BWCamera camera) {
            return hasTargets(camera) ? getLatestResult(camera).getBestTarget().getFiducialId() : 0.0;
        }

        public static double getBestTagDist(BW.BWCamera camera) {
            return hasTargets(camera)
                    ? getLatestResult(camera).getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d().getNorm()
                    : 0.0;
        }

        public static boolean isMultiTag(BW.BWCamera camera) {
            return getLatestResult(camera).getTargets().size() >= 2;
        }

        public static boolean isTagClear(BW.BWCamera camera) {
            return hasTargets(camera) && getBestTagDist(camera) < Constants.VisionConstants.PhotonConstants.BW_MAX_TAG_CLEAR_DIST;
        }

        public static boolean isTagClear() {
            return isTagClear(BW.BWCamera.TOP_RIGHT) || isTagClear(BW.BWCamera.TOP_LEFT) || isTagClear(BW.BWCamera.BACK);
        }

        public static Optional<EstimatedRobotPose> getMultiTagPose(BW.BWCamera camera) {
            Optional<MultiTargetPNPResult> multiTagResult = getLatestResult(camera).getMultiTagResult();
            AtomicReference<Optional<EstimatedRobotPose>> optionalPoseToReturn = new AtomicReference<>(Optional.empty());
            multiTagResult.ifPresent(
                    multiTargetPNPResult -> {
                        Pose3d bestPose = new Pose3d().plus(multiTargetPNPResult.estimatedPose.best).relativeTo(FieldUtil.ORIGIN).plus(getRobotToBWOffset(camera).inverse());
                        optionalPoseToReturn.set(Optional.of(new EstimatedRobotPose(
                                bestPose,
                                getLatestResultTimestamp(camera),
                                getLatestResult(camera).getTargets(),
                                Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION.apply(getBestTagDist(camera))
                        )));
                    }
            );
            return optionalPoseToReturn.get();
        }

        public static Optional<EstimatedRobotPose> getSingleTagPose(BW.BWCamera camera, Pose2d currentPose) {
            if (hasTargets(camera)) {
                Pose3d tagPose = FieldUtil.AprilTag.getTag(getBestTagID(camera)).pose3d;
                PhotonPipelineResult result = getLatestResult(camera);
                Transform3d cameraToTargetBest = result.getBestTarget().getBestCameraToTarget();
                Transform3d cameraToTargetAlt = result.getBestTarget().getAlternateCameraToTarget();

                double bestDist = cameraToTargetBest.getTranslation().toTranslation2d().getNorm();
                double altDist = cameraToTargetAlt.getTranslation().toTranslation2d().getNorm();

                Pose3d poseBest = tagPose.plus(cameraToTargetBest.inverse()).relativeTo(FieldUtil.ORIGIN).plus(getRobotToBWOffset(camera).inverse());
                Pose3d poseAlt = tagPose.plus(cameraToTargetAlt.inverse()).relativeTo(FieldUtil.ORIGIN).plus(getRobotToBWOffset(camera).inverse());

                Pose3d poseToReturn;
                double distToApply;

                // Disambiguate using the current pose i.e., choose the pose most consistent with the robot's current rotation
                double ambiguity = getLatestResult(camera).getBestTarget().getPoseAmbiguity();
                if (ambiguity < 0.15) {
                    poseToReturn = poseBest;
                    distToApply = bestDist;
                } else if (Math.abs(poseBest.toPose2d().getRotation().minus(currentPose.getRotation()).getDegrees())
                        < Math.abs(poseAlt.toPose2d().getRotation().minus(currentPose.getRotation()).getDegrees())
                        && ambiguity < 0.4) {
                    poseToReturn = poseBest;
                    distToApply = bestDist;
                } else if (ambiguity < 0.4) {
                    poseToReturn = poseAlt;
                    distToApply = altDist;
                } else {
                    return Optional.empty();
                }

                // Check if the pose is inside the field
                if (FieldUtil.isInField(poseToReturn)) {
                    return Optional.of(new EstimatedRobotPose(
                            poseToReturn,
                            result.getTimestampSeconds(),
                            result.getTargets(),
                            Constants.VisionConstants.VISION_STD_DEV_FUNCTION.apply(distToApply)
                    ));
                }
            }
            return Optional.empty();
        }

        public static Optional<EstimatedRobotPose> getBestTagPose(BW.BWCamera camera, Pose2d currentPose) {
            return BW.isMultiTag(camera) ? getMultiTagPose(camera) : getSingleTagPose(camera, currentPose);
        }

        public static void updateResults() {
            for (BW.BWCamera camera : BW.BWCamera.values()) {
                List<PhotonPipelineResult> results = getCamera(camera).getAllUnreadResults();

                if (!results.isEmpty()) {
                    switch (camera) {
                        case TOP_RIGHT -> latestResultTopRight = results.get(results.size() - 1);
                        case TOP_LEFT -> latestResultTopLeft = results.get(results.size() - 1);
                        case BACK -> latestResultBack = results.get(results.size() - 1);
                    }
                }
            }
        }
    }
}
