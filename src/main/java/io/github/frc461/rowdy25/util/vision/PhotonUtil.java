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

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.util.EstimatedRobotPose;
import io.github.frc461.rowdy25.util.FieldUtil;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

/**
 * Utility class for interfacing with PhotonVision, a vision processing solution that uses pipelines created with USB cameras. Contains methods to retrieve and process target data from multiple cameras (both color and black-and-white).
 *
 * @see <a href="https://docs.photonvision.org/en/latest/">PhotonVision Documentation</a> for more information about using PhotonVision and PhotonLib, the corresponding API library.
 * @author Eugene Zhang, <a href="https://github.com/e500">GitHub</a>
 */
public final class PhotonUtil {
    /**
     * Updates the latest vision results for both color and black-and-white cameras.
     *
     * @param heading The current heading of the robot.
     */
    public static void updateResults(Rotation2d heading) {
        Color.updateResults();
        BW.updateResults(heading);
    }

    /**
     * Nested utility class specifically for processing data from the color camera(s).
     */
    public static final class Color {
        /** Enum representing the specific target classes detectable by the color camera. */
        public enum TargetClass {
            /** Represents an Algae target. */
            ALGAE(0),
            /** Represents a Coral target. */
            CORAL(1),
            /** Represents no target. */
            NONE(-1);

            /** The ID of the target class. */
            public final int id;

            /**
             * Constructs a TargetClass with the specified ID.
             *
             * @param id The ID of the target class.
             */
            TargetClass(int id) {
                this.id = id;
            }

            /**
             * Returns the corresponding TargetClass for a given ID.
             *
             * @param id The ID of the target class.
             * @return The TargetClass associated with the ID. Returns NONE if the ID does not match a known class.
             */
            public static Color.TargetClass fromID(int id) {
                return switch (id) {
                    case 0 -> ALGAE;
                    case 1 -> CORAL;
                    default -> NONE;
                };
            }
        }

        /** The standard Color PhotonCamera instance. */
        private static final PhotonCamera COLOR = new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.PhotonConstants.COLOR_NAME);

        /** The transform representing the position of the color camera relative to the robot. */
        private static final Transform3d robotToCameraOffset = new Transform3d(
                Constants.VisionConstants.PhotonConstants.COLOR_FORWARD,
                Constants.VisionConstants.PhotonConstants.COLOR_LEFT,
                Constants.VisionConstants.PhotonConstants.COLOR_UP,
                new Rotation3d(
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.COLOR_ROLL),
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.COLOR_PITCH),
                        Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.COLOR_YAW)
                )
        );

        /** The latest result from the Color camera pipeline. */
        private static PhotonPipelineResult latestResult = new PhotonPipelineResult();

        /**
         * Checks whether the color camera detects any targets.
         *
         * @return A boolean representing whether any targets are currently detected.
         */
        public static boolean hasTargets() {
            return latestResult.hasTargets();
        }

        /**
         * Checks whether the color camera detects any targets of a specific class.
         *
         * @param targetClass The specific target class to look for.
         * @return A boolean representing whether targets of the specified class are detected.
         */
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

        // TODO SHOP: TEST ALGAE AND CORAL SPECIFIC TARGETING
        /**
         * Checks whether the color camera detects any algae targets.
         *
         * @return A boolean representing whether algae targets are detected.
         */
        public static boolean hasAlgaeTargets() {
            return hasTargets(Color.TargetClass.ALGAE);
        }

        /**
         * Checks whether the color camera detects any coral targets.
         *
         * @return A boolean representing whether coral targets are detected.
         */
        public static boolean hasCoralTargets() {
            return hasTargets(Color.TargetClass.CORAL);
        }

        /**
         * Retrieves the best tracking target detected by the color camera.
         *
         * @return An {@link Optional} containing the best detected object, or empty if none are detected.
         */
        public static Optional<PhotonTrackedTarget> getBestObject() {
            return hasTargets() ? Optional.of(latestResult.getBestTarget()) : Optional.empty();
        }

        /**
         * Retrieves the best tracking target of a specific class detected by the color camera.
         *
         * @param targetClass The specific target class to look for.
         * @return An {@link Optional} containing the best detected object of the given class, or empty if none are detected.
         */
        public static Optional<PhotonTrackedTarget> getBestObject(Color.TargetClass targetClass) {
            if (hasTargets(targetClass)) {
                for (PhotonTrackedTarget target : latestResult.getTargets()) {
                    if (target.getDetectedObjectClassID() == targetClass.id) {
                        return Optional.of(target);
                    }
                }
            }
            return Optional.empty();
        }

        /**
         * Retrieves the class of the best detected object.
         *
         * @return The best object's class, or {@link Color.TargetClass#NONE} if no object is detected.
         */
        public static Color.TargetClass getBestObjectClass() {
            return getBestObject().map(bestObject -> TargetClass.fromID(bestObject.getDetectedObjectClassID()))
                    .orElse(Color.TargetClass.NONE);
        }

        /**
         * Retrieves the yaw angle of the best detected object.
         *
         * @return The best object's yaw angle in degrees.
         */
        public static double getBestObjectYaw() {
            return getBestObject().map(PhotonTrackedTarget::getYaw).orElse(0.0);
        }

        /**
         * Retrieves the pitch angle of the best detected object.
         *
         * @return The best object's pitch angle in degrees.
         */
        public static double getBestObjectPitch() {
            return getBestObject().map(PhotonTrackedTarget::getPitch).orElse(0.0);
        }

        /**
         * Retrieves the yaw angle of the best detected object of a specific class.
         *
         * @param targetClass The specific target class to look for.
         * @return The yaw angle of the best object of the given class, or 0.0 if none are detected.
         */
        public static double getBestObjectYaw(Color.TargetClass targetClass) {
            return getBestObject(targetClass).map(PhotonTrackedTarget::getYaw).orElse(0.0);
        }

        /**
         * Retrieves the pitch angle of the best detected object of a specific class.
         *
         * @param targetClass The specific target class to look for.
         * @return The pitch angle of the best object of the given class, or 0.0 if none are detected.
         */
        public static double getBestObjectPitch(Color.TargetClass targetClass) {
            return getBestObject(targetClass).map(PhotonTrackedTarget::getPitch).orElse(0.0);
        }

        /**
         * Computes and returns the 2D translation from the robot to the best detected object of a specific class.
         *
         * @param targetClass The class of the object to target.
         * @return An {@link Optional} containing the {@link Translation2d} to the best object, or empty if none are found.
         */
        public static Optional<Translation2d> getRobotToBestObject(TargetClass targetClass) {
            if (!hasTargets(targetClass) || getBestObject(targetClass).isEmpty()) {
                return Optional.empty();
            }

            PhotonTrackedTarget bestTarget = getBestObject(targetClass).get();

            Translation2d camToObjectTranslation = new Pose3d(
                    Translation3d.kZero,
                    new Rotation3d(
                            0,
                            -Math.toRadians(bestTarget.getPitch()),
                            -Math.toRadians(bestTarget.getYaw())
                    )
            ).transformBy(
                    new Transform3d(
                            new Translation3d(bestTarget.getBestCameraToTarget().getTranslation().getNorm(), 0, 0),
                            Rotation3d.kZero
                    )
            ).getTranslation().rotateBy(
                    new Rotation3d(
                            robotToCameraOffset.getRotation().getX(),
                            robotToCameraOffset.getRotation().getY(),
                            0
                    )
            ).toTranslation2d();

            return Optional.of(new Translation2d(robotToCameraOffset.getX(), robotToCameraOffset.getY())
                    .plus(camToObjectTranslation.rotateBy(robotToCameraOffset.getRotation().toRotation2d().unaryMinus())));
        }

        /**
         * Updates the latest result using the last unread internal pipeline result and drains the rest of the unread results from the color camera pipeline(s).
         */
        public static void updateResults() {
            List<PhotonPipelineResult> results = COLOR.getAllUnreadResults();
            if (!results.isEmpty()) {
                latestResult = results.get(results.size() - 1);
            }
        }
    }

    /**
     * Nested utility class specifically for processing data from the black-and-white camera(s), as well as configuring them.
     */
    public static final class BW {
        /** The buffer storing historical robot heading data for localization latency compensation. */
        private static final TimeInterpolatableBuffer<Rotation2d> headingBuffer =
                TimeInterpolatableBuffer.createBuffer(1.0);

        /**
         * Enum representing the different black-and-white cameras available on the robot.
         */
        public enum BWCamera {
            /** Represents the top right black-and-white camera. */
            TOP_RIGHT(
                    new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_NAME),
                    new Transform3d(
                            Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_FORWARD,
                            Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_LEFT,
                            Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_UP,
                            new Rotation3d(
                                    Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_ROLL),
                                    Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_PITCH),
                                    Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_YAW)
                            )
                    )
            ),
            /** Represents the top left black-and-white camera. */
            TOP_LEFT(
                    new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_NAME),
                    new Transform3d(
                            Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_FORWARD,
                            Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_LEFT,
                            Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_UP,
                            new Rotation3d(
                                    Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_ROLL),
                                    Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_PITCH),
                                    Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_YAW)
                            )
                    )
            ),
            /** Represents the back black-and-white camera. */
            BACK(
                    new PhotonCamera(Constants.NT_INSTANCE, Constants.VisionConstants.PhotonConstants.BW_BACK_NAME),
                    new Transform3d(
                            Constants.VisionConstants.PhotonConstants.BW_BACK_FORWARD,
                            Constants.VisionConstants.PhotonConstants.BW_BACK_LEFT,
                            Constants.VisionConstants.PhotonConstants.BW_BACK_UP,
                            new Rotation3d(
                                    Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_BACK_ROLL),
                                    Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_BACK_PITCH),
                                    Units.degreesToRadians(Constants.VisionConstants.PhotonConstants.BW_BACK_YAW)
                            )
                    )
            );

            /** The PhotonCamera instance for this enum constant. */
            final PhotonCamera camera;
            /** The transform representing the position of the camera relative to the robot. */
            final Transform3d robotToCameraOffset;

            /**
             * Constructs a BWCamera enum constant.
             *
             * @param camera The PhotonCamera instance.
             * @param robotToCameraOffset The transform representing the camera's position relative to the robot.
             */
            BWCamera(PhotonCamera camera, Transform3d robotToCameraOffset) {
                this.camera = camera;
                this.robotToCameraOffset = robotToCameraOffset;
            }

            /**
             * Gets the PhotonCamera instance associated with this camera constant.
             *
             * @return The PhotonCamera instance.
             */
            public PhotonCamera getCamera() {
                return camera;
            }

            /**
             * Gets the transform representing the position of the camera relative to the robot.
             *
             * @return The robot-to-camera offset as a {@link Transform3d}.
             */
            public Transform3d getRobotToCameraOffset() {
                return robotToCameraOffset;
            }
        }

        /** The latest result from the top right black-and-white camera. */
        private static PhotonPipelineResult latestResultTopRight = new PhotonPipelineResult();
        /** The latest result from the top left black-and-white camera. */
        private static PhotonPipelineResult latestResultTopLeft = new PhotonPipelineResult();
        /** The latest result from the back black-and-white camera. */
        private static PhotonPipelineResult latestResultBack = new PhotonPipelineResult();

        /**
         * Retrieves the latest PhotonPipelineResult for the specified black-and-white camera.
         *
         * @param camera The specific black-and-white camera to read from.
         * @return The latest PhotonPipelineResult for the camera.
         */
        public static PhotonPipelineResult getLatestResult(BWCamera camera) {
            return switch (camera) {
                case TOP_RIGHT -> latestResultTopRight;
                case TOP_LEFT -> latestResultTopLeft;
                case BACK -> latestResultBack;
            };
        }

        /**
         * Checks whether a specified black-and-white camera detects any AprilTags.
         *
         * @param camera The specified camera.
         * @return True if targets are present, false otherwise.
         */
        public static boolean hasTargets(BWCamera camera) {
            return switch (camera) {
                case TOP_RIGHT -> latestResultTopRight.hasTargets();
                case TOP_LEFT -> latestResultTopLeft.hasTargets();
                case BACK -> latestResultBack.hasTargets();
            };
        }

        /**
         * Retrieves the timestamp of the latest result from a specified black-and-white camera.
         *
         * @param camera The specified camera.
         * @return The timestamp of the latest result, in seconds.
         */
        public static double getLatestResultTimestamp(BWCamera camera) {
            return switch (camera) {
                case TOP_RIGHT -> latestResultTopRight.getTimestampSeconds();
                case TOP_LEFT -> latestResultTopLeft.getTimestampSeconds();
                case BACK -> latestResultBack.getTimestampSeconds();
            };
        }

        /**
         * Retrieves the ID of the best observed AprilTag from the specified black-and-white camera.
         *
         * @param camera The specified camera.
         * @return The ID of the best AprilTag, or 0.0 if none.
         */
        public static double getBestTagID(BWCamera camera) {
            return hasTargets(camera) ? getLatestResult(camera).getBestTarget().getFiducialId() : 0.0;
        }

        /**
         * Retrieves the distance from the camera to the best tag detected by the given camera.
         *
         * @param camera The specified camera.
         * @return The horizontal distance in meters, or 0.0 if no tag is found.
         */
        public static double getBestTagDist(BWCamera camera) {
            return hasTargets(camera)
                    ? getLatestResult(camera).getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d().getNorm()
                    : 0.0;
        }

        /**
         * Checks whether the specified black-and-white camera detects at least two AprilTags.
         *
         * @param camera The specified camera.
         * @return True if multiple tags are seen, false otherwise.
         */
        public static boolean isMultiTag(BWCamera camera) {
            return getLatestResult(camera).getTargets().size() >= 2;
        }

        /**
         * Checks whether a detected tag is close enough to be considered reliable (clear).
         *
         * @param camera The specific camera.
         * @return True if the nearest tag is within the valid distance threshold.
         */
        public static boolean isTagClear(BWCamera camera) {
            return hasTargets(camera) && getBestTagDist(camera) < Constants.VisionConstants.PhotonConstants.BW_MAX_TAG_CLEAR_DIST;
        }

        /**
         * Checks whether any black-and-white camera on the robot has a reliably clear AprilTag.
         *
         * @return True if at least one camera detects a tag within the valid distance threshold.
         */
        public static boolean isTagClear() {
            return isTagClear(BWCamera.TOP_RIGHT) || isTagClear(BWCamera.TOP_LEFT) || isTagClear(BWCamera.BACK);
        }

        /**
         * Computes the estimated robot pose using multiple tags from the specified camera.
         *
         * @param camera The specified camera.
         * @return An {@link Optional} containing the estimated robot pose, or empty if computation fails.
         */
        public static Optional<EstimatedRobotPose> getMultiTagPose(BWCamera camera) {
            Optional<MultiTargetPNPResult> multiTagResult = getLatestResult(camera).getMultiTagResult();
            return multiTagResult.map(
                    multiTargetPNPResult -> {
                        Pose3d bestPose = new Pose3d().plus(multiTargetPNPResult.estimatedPose.best).relativeTo(FieldUtil.ORIGIN).plus(camera.robotToCameraOffset.inverse());
                        return new EstimatedRobotPose(
                                bestPose,
                                getLatestResultTimestamp(camera),
                                getLatestResult(camera).getTargets(),
                                Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION.apply(getBestTagDist(camera))
                        );
                    }
            );
        }

        /**
         * Computes the estimated robot pose using a single tag from the specified camera, incorporating the robot's current pose for disambiguation.
         *
         * @param camera The specified camera.
         * @param currentPose The robot's current pose.
         * @return An {@link Optional} containing the estimated robot pose, or empty if computation fails.
         */
        public static Optional<EstimatedRobotPose> getSingleTagPose(BWCamera camera, Pose2d currentPose) {
            if (!hasTargets(camera)) {
                return Optional.empty();
            }

            Pose3d tagPose = FieldUtil.AprilTag.getTag(getBestTagID(camera)).pose3d;
            PhotonPipelineResult result = getLatestResult(camera);

            Transform3d cameraToTargetBest = result.getBestTarget().getBestCameraToTarget();
            Transform3d cameraToTargetAlt = result.getBestTarget().getAlternateCameraToTarget();

            double bestDist = cameraToTargetBest.getTranslation().toTranslation2d().getNorm();
            double altDist = cameraToTargetAlt.getTranslation().toTranslation2d().getNorm();

            Pose3d poseBest = tagPose.plus(cameraToTargetBest.inverse()).relativeTo(FieldUtil.ORIGIN).plus(camera.robotToCameraOffset.inverse());
            Pose3d poseAlt = tagPose.plus(cameraToTargetAlt.inverse()).relativeTo(FieldUtil.ORIGIN).plus(camera.robotToCameraOffset.inverse());

            Pose3d poseToReturn;
            double distToApply;

            // Disambiguate using the current pose, that is, choose the pose most consistent with the robot's current rotation
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
            if (!FieldUtil.isInField(poseToReturn)) {
                return Optional.empty();
            }

            return Optional.of(new EstimatedRobotPose(
                    poseToReturn,
                    result.getTimestampSeconds(),
                    result.getTargets(),
                    Constants.VisionConstants.VISION_STD_DEV_FUNCTION.apply(distToApply)
            ));
        }

        /**
         * Computes the estimated robot pose using a single tag from the specified camera, utilizing the historical heading buffer. Known to have better accuracy due to a simple algorithm that does not require disambiguation.
         *
         * @param camera The specified camera.
         * @return An {@link Optional} containing the estimated robot pose, or empty if computation fails.
         */
        private static Optional<EstimatedRobotPose> getSingleTagPose(BWCamera camera) {
            if (!hasTargets(camera)) {
                return Optional.empty();
            }

            PhotonPipelineResult result = getLatestResult(camera);
            PhotonTrackedTarget bestTarget = result.getBestTarget();

            Translation2d camToTagTranslation = new Pose3d(
                    Translation3d.kZero,
                    new Rotation3d(
                            0,
                            -Math.toRadians(bestTarget.getPitch()),
                            -Math.toRadians(bestTarget.getYaw())
                    )
            ).transformBy(
                    new Transform3d(
                            new Translation3d(bestTarget.getBestCameraToTarget().getTranslation().getNorm(), 0, 0),
                            Rotation3d.kZero
                    )
            ).getTranslation().rotateBy(
                    new Rotation3d(
                            camera.robotToCameraOffset.getRotation().getX(),
                            camera.robotToCameraOffset.getRotation().getY(),
                            0
                    )
            ).toTranslation2d();

            if (headingBuffer.getSample(result.getTimestampSeconds()).isEmpty()) {
                return Optional.empty();
            }

            Rotation2d headingSample = headingBuffer.getSample(result.getTimestampSeconds()).get();

            Rotation2d camToTagRotation = headingSample.plus(
                    camera.robotToCameraOffset.getRotation().toRotation2d().plus(camToTagTranslation.getAngle())
            );

            if (FieldUtil.layout2025.getTagPose(bestTarget.getFiducialId()).isEmpty()) {
                return Optional.empty();
            }

            FieldUtil.AprilTag tag = FieldUtil.AprilTag.getTag(bestTarget.getFiducialId());

            if (!FieldUtil.AprilTag.FILTER.contains(tag)) {
                return Optional.empty();
            }

            Pose2d tagPose2d = tag.pose2d;

            Translation2d fieldToCameraTranslation =
                    new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                            .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0, Rotation2d.kZero))
                            .getTranslation();

            Pose2d robotPose = new Pose2d(
                    new Pose2d(
                            fieldToCameraTranslation,
                            headingSample.plus(camera.robotToCameraOffset.getRotation().toRotation2d())
                    ).transformBy(
                            new Transform2d(
                                    new Pose3d(
                                            camera.robotToCameraOffset.getTranslation(),
                                            camera.robotToCameraOffset.getRotation()
                                    ).toPose2d(),
                                    Pose2d.kZero
                            )
                    ).getTranslation(),
                    headingSample
            );

            return Optional.of(
                    new EstimatedRobotPose(
                            new Pose3d(robotPose),
                            result.getTimestampSeconds(),
                            result.getTargets(),
                            Constants.VisionConstants.VISION_STD_DEV_FUNCTION.apply(camToTagTranslation.getNorm())
                    )
            );
        }

        /**
         * Computes the best estimate of the robot's pose based on the most reliable tag information from the specified camera.
         *
         * @param camera The specified camera.
         * @return An {@link Optional} containing the best estimated robot pose, or empty if no valid pose can be computed.
         */
        public static Optional<EstimatedRobotPose> getBestTagPose(BWCamera camera) {
            return BW.isMultiTag(camera) ? getMultiTagPose(camera) : getSingleTagPose(camera);
        }

        /**
         * Updates the latest result using the last unread internal pipeline result and the robot's heading, and drains the rest of the unread results from the black-and-white camera pipeline(s).
         *
         * @param heading The current heading of the robot.
         */
        public static void updateResults(Rotation2d heading) {
            headingBuffer.addSample(Timer.getFPGATimestamp(), heading);
            for (BWCamera camera : BWCamera.values()) {
                List<PhotonPipelineResult> results = camera.camera.getAllUnreadResults();

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
