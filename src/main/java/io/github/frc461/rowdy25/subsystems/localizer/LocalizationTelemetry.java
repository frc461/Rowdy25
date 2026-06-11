package io.github.frc461.rowdy25.subsystems.localizer;

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

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import io.github.frc461.rowdy25.RobotStates;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.util.vision.LimelightUtil;
import io.github.frc461.rowdy25.util.vision.PhotonUtil;

/**
 * Telemetry publisher for the localization subsystem.
 * <p>
 * Publishes localization state (estimated poses, strategy, distances to targets,
 * Limelight/Photon camera data) to NetworkTables, DogLog, and SmartDashboard Field2d.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 */
public class LocalizationTelemetry {
    /** Possible QuestNav fault states. */
    public enum QuestFault {
        /** Quest headset battery is low. */
        QUEST_LOW_BATTERY,
        /** Quest headset has powered off. */
        QUEST_DIED,
        /** Quest headset is disconnected. */
        QUEST_DISCONNECTED
    }

    /** The localizer subsystem to read state from. */
    private final Localizer localizer;

    /**
     * Constructs a LocalizationTelemetry instance.
     *
     * @param localizer The localizer subsystem to read state from.
     */
    public LocalizationTelemetry(Localizer localizer) {
        this.localizer = localizer;
    }

    /** NetworkTable for general localization telemetry. */
    private final NetworkTable localizationTelemetryTable = Constants.NT_INSTANCE.getTable("LocalizationTelemetry");

    /** NetworkTable for Limelight camera telemetry. */
    private final NetworkTable limelightTelemetryTable = Constants.NT_INSTANCE.getTable("LimelightTelemetry");

    /** NetworkTable for PhotonVision camera telemetry. */
    private final NetworkTable photonTelemetryTable = Constants.NT_INSTANCE.getTable("PhotonTelemetry");

    /** Publisher for the estimated pose as a formatted string. */
    private final StringPublisher poseEstimatePrettyPub = localizationTelemetryTable.getStringTopic("Estimated Pose").publish();

    /** Publisher for the temporary target pose as a formatted string. */
    private final StringPublisher temporaryTargetPosePrettyPub = localizationTelemetryTable.getStringTopic("Temporary Target Pose").publish();

    /** Publisher for the nearest robot pose at a reef branch as a formatted string. */
    private final StringPublisher nearestRobotPoseAtBranchPrettyPub = localizationTelemetryTable.getStringTopic("Nearest Branch Pose April Tag Offset").publish();

    /** Publisher for the QuestNav-based pose as a formatted string. */
    private final StringPublisher questPosePrettyPub = localizationTelemetryTable.getStringTopic("Quest-Based Pose").publish();

    /** Publisher for the current localization strategy name. */
    private final StringPublisher localizationStrategyPub = localizationTelemetryTable.getStringTopic("Localization Strategy").publish();

    /** Publisher for the distance to the coral scoring location. */
    private final DoublePublisher distanceToCoralScoringLocation = localizationTelemetryTable.getDoubleTopic("DistanceToCoralScoringLocation").publish();

    /** Publisher for the distance to the coral station. */
    private final DoublePublisher distanceToCoralStation = localizationTelemetryTable.getDoubleTopic("DistanceToCoralStation").publish();

    /** Publisher for whether the robot is at the coral scoring location. */
    private final BooleanPublisher atCoralScoringLocation = localizationTelemetryTable.getBooleanTopic("AtScoringLocation").publish();

    /** Publisher for whether the robot is against the reef wall. */
    private final BooleanPublisher againstReef = localizationTelemetryTable.getBooleanTopic("AgainstReef").publish();

    /** Publisher for whether the robot is against the coral station. */
    private final BooleanPublisher againstCoralStation = localizationTelemetryTable.getBooleanTopic("AgainstCoralStation").publish();

    /** Publisher for the Limelight MegaTagOne pose as a formatted string. */
    private final StringPublisher megaTagOnePosePrettyPub = limelightTelemetryTable.getStringTopic("MegaTagOne Pose").publish();

    /** Publisher for the Limelight MegaTagTwo pose as a formatted string. */
    private final StringPublisher megaTagTwoPosePrettyPub = limelightTelemetryTable.getStringTopic("MegaTagTwo Pose").publish();

    /** Publisher for the nearest AprilTag distance from Limelight. */
    private final DoublePublisher nearestTagDistPub = limelightTelemetryTable.getDoubleTopic("Nearest Tag Distance").publish();

    /** Publisher for whether Limelight measurements are being added to the estimator. */
    private final BooleanPublisher canAddLLMeasurementsPub = limelightTelemetryTable.getBooleanTopic("Adding Limelight Measurements").publish();

    /** Publisher for whether the color camera has an algae target. */
    private final BooleanPublisher photonColorHasAlgaeTargetPub = photonTelemetryTable.getBooleanTopic("Photon Color Has Algae Target").publish();

    /** Publisher for whether the color camera has a coral target. */
    private final BooleanPublisher photonColorHasCoralTargetPub = photonTelemetryTable.getBooleanTopic("Photon Color Has Coral Target").publish();

    /** Publisher for the best object class detected by the color camera. */
    private final StringPublisher photonColorBestObjectClass = photonTelemetryTable.getStringTopic("Photon Color Best Object Class").publish();

    /** Publisher for the color camera best object pose as a formatted string. */
    private final StringPublisher photonColorBestObjectPosePrettyPub = photonTelemetryTable.getStringTopic("Photon Color Best Object Pose").publish();

    /** Publisher for the top-right BW camera pose as a formatted string. */
    private final StringPublisher photonTopRightPosePrettyPub = photonTelemetryTable.getStringTopic("Photon Top Right Pose").publish();

    /** Publisher for the top-left BW camera pose as a formatted string. */
    private final StringPublisher photonTopLeftPosePrettyPub = photonTelemetryTable.getStringTopic("Photon Top Left Pose").publish();

    /** Publisher for the back BW camera pose as a formatted string. */
    private final StringPublisher photonBackPosePrettyPub = photonTelemetryTable.getStringTopic("Photon Back Pose").publish();

    /** Publisher for whether the top-right camera is adding measurements. */
    private final BooleanPublisher canAddTopRightMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Top Right Measurements").publish();

    /** Publisher for whether the top-left camera is adding measurements. */
    private final BooleanPublisher canAddTopLeftMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Top Left Measurements").publish();

    /** Publisher for whether the back camera is adding measurements. */
    private final BooleanPublisher canAddBackMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Back Measurements").publish();

    /** NetworkTable for Field2d pose visualization. */
    private final NetworkTable robotPoseTable = Constants.NT_INSTANCE.getTable("Pose");

    /** Publisher for the Field2d type identifier. */
    private final StringPublisher fieldTypePub = robotPoseTable.getStringTopic(".type").publish();

    /** Publisher for the estimated pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> pose2dEstimatePub = robotPoseTable.getStructTopic("Estimated Pose2d", Pose2d.struct).publish();

    /** Publisher for the estimated pose as a double array. */
    private final DoubleArrayPublisher poseEstimatePub = robotPoseTable.getDoubleArrayTopic("Estimated Pose").publish();

    /** Publisher for the temporary target pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> temporaryTargetPose2dPub = robotPoseTable.getStructTopic("Temporary Target Pose2d", Pose2d.struct).publish();

    /** Publisher for the temporary target pose as a double array. */
    private final DoubleArrayPublisher temporaryTargetPosePub = robotPoseTable.getDoubleArrayTopic("Temporary Target Pose").publish();

    /** Publisher for the nearest branch pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> nearestRobotPoseAtBranchPose2dPub = robotPoseTable.getStructTopic("Nearest Branch April Tag Offset Pose2d", Pose2d.struct).publish();

    /** Publisher for the nearest branch pose as a double array. */
    private final DoubleArrayPublisher nearestRobotPoseAtBranchPosePub = robotPoseTable.getDoubleArrayTopic("Nearest Branch April Tag Offset Pose").publish();

    /** Publisher for the QuestNav pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> questPose2dPub = robotPoseTable.getStructTopic("Quest-Based Pose2d", Pose2d.struct).publish();

    /** Publisher for the QuestNav pose as a double array. */
    private final DoubleArrayPublisher questPosePub = robotPoseTable.getDoubleArrayTopic("Quest-Based Pose").publish();

    /** Publisher for the Limelight MegaTagOne pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> megaTagOnePose2dPub = robotPoseTable.getStructTopic("MegaTagOne Pose2d", Pose2d.struct).publish();

    /** Publisher for the Limelight MegaTagOne pose as a double array. */
    private final DoubleArrayPublisher megaTagOnePosePub = robotPoseTable.getDoubleArrayTopic("MegaTagOne Pose").publish();

    /** Publisher for the Limelight MegaTagTwo pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> megaTagTwoPose2dPub = robotPoseTable.getStructTopic("MegaTagTwo Pose2d", Pose2d.struct).publish();

    /** Publisher for the Limelight MegaTagTwo pose as a double array. */
    private final DoubleArrayPublisher megaTagTwoPosePub = robotPoseTable.getDoubleArrayTopic("MegaTagTwo Pose").publish();

    /** Publisher for the color camera best object pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> photonColorBestObjectPose2dPub = robotPoseTable.getStructTopic("Photon Color Best Object Pose2d", Pose2d.struct).publish();

    /** Publisher for the color camera best object pose as a double array. */
    private final DoubleArrayPublisher photonColorBestObjectPosePub = robotPoseTable.getDoubleArrayTopic("Photon Color Best Object Pose").publish();

    /** Publisher for the top-right BW camera pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> photonTopRightPose2dPub = robotPoseTable.getStructTopic("Photon Top Right Pose2d", Pose2d.struct).publish();

    /** Publisher for the top-right BW camera pose as a double array. */
    private final DoubleArrayPublisher photonTopRightPosePub = robotPoseTable.getDoubleArrayTopic("Photon Top Right Pose").publish();

    /** Publisher for the top-left BW camera pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> photonTopLeftPose2dPub = robotPoseTable.getStructTopic("Photon Top Left Pose2d", Pose2d.struct).publish();

    /** Publisher for the top-left BW camera pose as a double array. */
    private final DoubleArrayPublisher photonTopLeftPosePub = robotPoseTable.getDoubleArrayTopic("Photon Top Left Pose").publish();

    /** Publisher for the back BW camera pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> photonBackPose2dPub = robotPoseTable.getStructTopic("Photon Back Pose2d", Pose2d.struct).publish();

    /** Publisher for the back BW camera pose as a double array. */
    private final DoubleArrayPublisher photonBackPosePub = robotPoseTable.getDoubleArrayTopic("Photon Back Pose").publish();

    /** Publisher for the nearest robot pose at the coral station as a Pose2d struct. */
    private final StructPublisher<Pose2d> nearestRobotPoseAtCoralStation = robotPoseTable.getStructTopic("Nearest Robot Pose At Coral Station", Pose2d.struct).publish();

    /**
     * Publishes all localization telemetry values to NetworkTables, DogLog, and Field2d.
     */
    public void publishValues() {
        localizationStrategyPub.set(localizer.getLocalizationStrategy());

        nearestTagDistPub.set(LimelightUtil.getNearestTagDist());
        canAddLLMeasurementsPub.set(LimelightUtil.isTagClear());
        distanceToCoralScoringLocation.set(localizer.getDistanceToActionLocation(RobotStates.State.L4_CORAL));
        distanceToCoralStation.set(localizer.getDistanceToActionLocation(RobotStates.State.CORAL_STATION));
        atCoralScoringLocation.set(localizer.atScoringLocation(RobotStates.State.L4_CORAL));
        againstReef.set(localizer.isAgainstReefWall());
        againstCoralStation.set(localizer.isAgainstCoralStation());

        photonColorHasAlgaeTargetPub.set(PhotonUtil.Color.hasAlgaeTargets());
        photonColorHasCoralTargetPub.set(PhotonUtil.Color.hasCoralTargets());
        photonColorBestObjectClass.set(PhotonUtil.Color.getBestObjectClass().name());
        canAddTopRightMeasurementsPub.set(PhotonUtil.BW.isTagClear(PhotonUtil.BW.BWCamera.TOP_RIGHT));
        canAddTopLeftMeasurementsPub.set(PhotonUtil.BW.isTagClear(PhotonUtil.BW.BWCamera.TOP_LEFT));
        canAddBackMeasurementsPub.set(PhotonUtil.BW.isTagClear(PhotonUtil.BW.BWCamera.BACK));

        fieldTypePub.set("Field2d");
        publishPose(pose2dEstimatePub, poseEstimatePub, poseEstimatePrettyPub, localizer.getEstimatedPose());
        publishPose(temporaryTargetPose2dPub, temporaryTargetPosePub, temporaryTargetPosePrettyPub, localizer.getCurrentTemporaryTargetPose());
        publishPose(nearestRobotPoseAtBranchPose2dPub, nearestRobotPoseAtBranchPosePub, nearestRobotPoseAtBranchPrettyPub, localizer.nearestRobotPoseAtBranch);
        publishPose(questPose2dPub, questPosePub, questPosePrettyPub, localizer.getQuestPose());
        publishPose(megaTagOnePose2dPub, megaTagOnePosePub, megaTagOnePosePrettyPub, LimelightUtil.getMegaTagOnePose());
        publishPose(megaTagTwoPose2dPub, megaTagTwoPosePub, megaTagTwoPosePrettyPub, LimelightUtil.getMegaTagTwoPose());
        publishPose(photonColorBestObjectPose2dPub, photonColorBestObjectPosePub, photonColorBestObjectPosePrettyPub, localizer.bestCoralPose);
        PhotonUtil.BW.getBestTagPose(PhotonUtil.BW.BWCamera.TOP_RIGHT).ifPresent(
                poseEstimate -> publishPose(photonTopRightPose2dPub, photonTopRightPosePub, photonTopRightPosePrettyPub, poseEstimate.estimatedPose().toPose2d())
        );
        PhotonUtil.BW.getBestTagPose(PhotonUtil.BW.BWCamera.TOP_LEFT).ifPresent(
                poseEstimate -> publishPose(photonTopLeftPose2dPub, photonTopLeftPosePub, photonTopLeftPosePrettyPub, poseEstimate.estimatedPose().toPose2d())
        );
        PhotonUtil.BW.getBestTagPose(PhotonUtil.BW.BWCamera.BACK).ifPresent(
                poseEstimate -> publishPose(photonBackPose2dPub, photonBackPosePub, photonBackPosePrettyPub, poseEstimate.estimatedPose().toPose2d())
        );
        nearestRobotPoseAtCoralStation.set(localizer.nearestRobotPoseAtCoralStation);

        logValues();
    }

    /** Logs localization state values to DogLog. */
    private void logValues() {
        DogLog.log("PoseEstimate", localizer.getEstimatedPose());
        DogLog.log("LocalizationStrategy", localizer.getLocalizationStrategy());
        PhotonUtil.BW.getBestTagPose(PhotonUtil.BW.BWCamera.TOP_RIGHT).ifPresent(
                poseEstimate -> DogLog.log("PhotonTopRightPose", poseEstimate.estimatedPose().toPose2d())
        );
        PhotonUtil.BW.getBestTagPose(PhotonUtil.BW.BWCamera.TOP_LEFT).ifPresent(
                poseEstimate -> DogLog.log("PhotonTopLeftPose", poseEstimate.estimatedPose().toPose2d())
        );
        PhotonUtil.BW.getBestTagPose(PhotonUtil.BW.BWCamera.BACK).ifPresent(
                poseEstimate -> DogLog.log("PhotonBackPose", poseEstimate.estimatedPose().toPose2d())
        );
        DogLog.log("PhotonColorHasTarget", PhotonUtil.Color.hasTargets());
        DogLog.log("PhotonBWTopRightHasTarget", PhotonUtil.BW.hasTargets(PhotonUtil.BW.BWCamera.TOP_RIGHT));
        DogLog.log("PhotonBWTopLeftHasTarget", PhotonUtil.BW.hasTargets(PhotonUtil.BW.BWCamera.TOP_LEFT));
        DogLog.log("PhotonBWBackHasTarget", PhotonUtil.BW.hasTargets(PhotonUtil.BW.BWCamera.BACK));
    }

    /**
     * Publishes a pose to both a Pose2d struct publisher, a double array publisher,
     * and a formatted string publisher.
     *
     * @param structPub The Pose2d struct publisher.
     * @param arrayPub The double array publisher.
     * @param prettyPub The formatted string publisher.
     * @param pose The pose to publish.
     */
    public void publishPose(StructPublisher<Pose2d> structPub, DoubleArrayPublisher arrayPub, StringPublisher prettyPub, Pose2d pose) {
        structPub.set(pose);
        arrayPub.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
        prettyPub.set("X: " + pose.getX() + ", Y: " + pose.getY() + ", Yaw: " + pose.getRotation().getDegrees());
    }
}