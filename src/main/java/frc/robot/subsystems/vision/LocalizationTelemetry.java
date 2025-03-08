package frc.robot.subsystems.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import frc.robot.RobotStates;
import frc.robot.constants.Constants;
import frc.robot.util.vision.LimelightUtil;
import frc.robot.util.vision.PhotonUtil;

public class LocalizationTelemetry {
    public enum QuestFault {
        QUEST_LOW_BATTERY,
        QUEST_DIED,
        QUEST_DISCONNECTED
    }

    private final Localizer localizer;

    public LocalizationTelemetry(Localizer localizer) {
        this.localizer = localizer;
    }

    private final NetworkTable localizationTelemetryTable = Constants.NT_INSTANCE.getTable("LocalizationTelemetry");
    private final NetworkTable limelightTelemetryTable = Constants.NT_INSTANCE.getTable("LimelightTelemetry");
    private final NetworkTable photonTelemetryTable = Constants.NT_INSTANCE.getTable("PhotonTelemetry");

    private final StringPublisher poseEstimatePrettyPub = localizationTelemetryTable.getStringTopic("Estimated Pose").publish();
    private final StringPublisher temporaryTargetPosePrettyPub = localizationTelemetryTable.getStringTopic("Temporary Target Pose").publish();
    private final StringPublisher questPosePrettyPub = localizationTelemetryTable.getStringTopic("Quest-Based Pose").publish();
    private final StringPublisher localizationStrategyPub = localizationTelemetryTable.getStringTopic("Localization Strategy").publish();
    private final DoublePublisher distanceToCoralScoringLocation = localizationTelemetryTable.getDoubleTopic("DistanceToCoralScoringLocation").publish();
    private final BooleanPublisher atCoralScoringLocation = localizationTelemetryTable.getBooleanTopic("AtScoringLocation").publish();

    private final StringPublisher megaTagOnePosePrettyPub = limelightTelemetryTable.getStringTopic("MegaTagOne Pose").publish();
    private final StringPublisher megaTagTwoPosePrettyPub = limelightTelemetryTable.getStringTopic("MegaTagTwo Pose").publish();
    private final DoublePublisher nearestTagDistPub = limelightTelemetryTable.getDoubleTopic("Nearest Tag Distance").publish();
    private final BooleanPublisher canAddLLMeasurementsPub = limelightTelemetryTable.getBooleanTopic("Adding Limelight Measurements").publish();

    private final BooleanPublisher photonColorHasAlgaeTargetPub = photonTelemetryTable.getBooleanTopic("Photon Color Has Algae Target").publish();
    private final BooleanPublisher photonColorHasCoralTargetPub = photonTelemetryTable.getBooleanTopic("Photon Color Has Coral Target").publish();
    private final StringPublisher photonColorBestObjectClass = photonTelemetryTable.getStringTopic("Photon Color Best Object Class").publish();
    private final StringPublisher photonTopRightPosePrettyPub = photonTelemetryTable.getStringTopic("Photon Top Right Pose").publish();
    private final StringPublisher photonTopLeftPosePrettyPub = photonTelemetryTable.getStringTopic("Photon Top Left Pose").publish();
    private final StringPublisher photonBackPosePrettyPub = photonTelemetryTable.getStringTopic("Photon Back Pose").publish();
    private final BooleanPublisher canAddTopRightMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Top Right Measurements").publish();
    private final BooleanPublisher canAddTopLeftMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Top Left Measurements").publish();
    private final BooleanPublisher canAddBackMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Back Measurements").publish();

    private final NetworkTable robotPoseTable = Constants.NT_INSTANCE.getTable("Pose");
    private final StringPublisher fieldTypePub = robotPoseTable.getStringTopic(".type").publish();
    private final StructPublisher<Pose2d> pose2dEstimatePub = robotPoseTable.getStructTopic("Estimated Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher poseEstimatePub = robotPoseTable.getDoubleArrayTopic("Estimated Pose").publish();
    private final StructPublisher<Pose2d> temporaryTargetPose2dPub = robotPoseTable.getStructTopic("Temporary Target Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher temporaryTargetPosePub = robotPoseTable.getDoubleArrayTopic("Temporary Target Pose").publish();
    private final StructPublisher<Pose2d> questPose2dPub = robotPoseTable.getStructTopic("Quest-Based Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher questPosePub = robotPoseTable.getDoubleArrayTopic("Quest-Based Pose").publish();
    private final StructPublisher<Pose2d> megaTagOnePose2dPub = robotPoseTable.getStructTopic("MegaTagOne Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher megaTagOnePosePub = robotPoseTable.getDoubleArrayTopic("MegaTagOne Pose").publish();
    private final StructPublisher<Pose2d> megaTagTwoPose2dPub = robotPoseTable.getStructTopic("MegaTagTwo Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher megaTagTwoPosePub = robotPoseTable.getDoubleArrayTopic("MegaTagTwo Pose").publish();
    private final StructPublisher<Pose2d> photonTopRightPose2dPub = robotPoseTable.getStructTopic("Photon Top Right Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher photonTopRightPosePub = robotPoseTable.getDoubleArrayTopic("Photon Top Right Pose").publish();
    private final StructPublisher<Pose2d> photonTopLeftPose2dPub = robotPoseTable.getStructTopic("Photon Top Left Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher photonTopLeftPosePub = robotPoseTable.getDoubleArrayTopic("Photon Top Left Pose").publish();
    private final StructPublisher<Pose2d> photonBackPose2dPub = robotPoseTable.getStructTopic("Photon Back Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher photonBackPosePub = robotPoseTable.getDoubleArrayTopic("Photon Back Pose").publish();

    public void publishValues() {
        localizationStrategyPub.set(localizer.getLocalizationStrategy());

        nearestTagDistPub.set(LimelightUtil.getNearestTagDist());
        canAddLLMeasurementsPub.set(LimelightUtil.isTagClear());
        distanceToCoralScoringLocation.set(localizer.getDistanceToScoringLocation(RobotStates.State.L4_CORAL));
        atCoralScoringLocation.set(localizer.atScoringLocation(RobotStates.State.L4_CORAL));

        photonColorHasAlgaeTargetPub.set(PhotonUtil.Color.hasAlgaeTargets());
        photonColorHasCoralTargetPub.set(PhotonUtil.Color.hasCoralTargets());
        photonColorBestObjectClass.set(PhotonUtil.Color.getBestObjectClass().name());
        canAddTopRightMeasurementsPub.set(PhotonUtil.BW.isTagClear(PhotonUtil.BW.BWCamera.TOP_RIGHT));
        canAddTopLeftMeasurementsPub.set(PhotonUtil.BW.isTagClear(PhotonUtil.BW.BWCamera.TOP_LEFT));
        canAddBackMeasurementsPub.set(PhotonUtil.BW.isTagClear(PhotonUtil.BW.BWCamera.BACK));


        fieldTypePub.set("Field2d");
        publishPose(pose2dEstimatePub, poseEstimatePub, poseEstimatePrettyPub, localizer.getEstimatedPose());
        publishPose(temporaryTargetPose2dPub, temporaryTargetPosePub, temporaryTargetPosePrettyPub, localizer.getCurrentTemporaryTargetPose());
        publishPose(questPose2dPub, questPosePub, questPosePrettyPub, localizer.getQuestPose());
        publishPose(megaTagOnePose2dPub, megaTagOnePosePub, megaTagOnePosePrettyPub, LimelightUtil.getMegaTagOnePose());
        publishPose(megaTagTwoPose2dPub, megaTagTwoPosePub, megaTagTwoPosePrettyPub, LimelightUtil.getMegaTagTwoPose());
        localizer.getUpdatedPhotonPoseEstimate(PhotonUtil.BW.BWCamera.TOP_RIGHT).ifPresent(
                poseEstimate -> publishPose(photonTopRightPose2dPub, photonTopRightPosePub, photonTopRightPosePrettyPub, poseEstimate.estimatedPose().toPose2d())
        );
        localizer.getUpdatedPhotonPoseEstimate(PhotonUtil.BW.BWCamera.TOP_LEFT).ifPresent(
                poseEstimate -> publishPose(photonTopLeftPose2dPub, photonTopLeftPosePub, photonTopLeftPosePrettyPub, poseEstimate.estimatedPose().toPose2d())
        );
        localizer.getUpdatedPhotonPoseEstimate(PhotonUtil.BW.BWCamera.BACK).ifPresent(
                poseEstimate -> publishPose(photonBackPose2dPub, photonBackPosePub, photonBackPosePrettyPub, poseEstimate.estimatedPose().toPose2d())
        );

        logValues();
    }

    private void logValues() {
        DogLog.log("PoseEstimate", localizer.getEstimatedPose());
        DogLog.log("LocalizationStrategy", localizer.getLocalizationStrategy());
        localizer.getUpdatedPhotonPoseEstimate(PhotonUtil.BW.BWCamera.TOP_RIGHT).ifPresent(
                poseEstimate -> DogLog.log("PhotonTopRightPose", poseEstimate.estimatedPose().toPose2d())
        );
        localizer.getUpdatedPhotonPoseEstimate(PhotonUtil.BW.BWCamera.TOP_LEFT).ifPresent(
                poseEstimate -> DogLog.log("PhotonTopLeftPose", poseEstimate.estimatedPose().toPose2d())
        );
        localizer.getUpdatedPhotonPoseEstimate(PhotonUtil.BW.BWCamera.BACK).ifPresent(
                poseEstimate -> DogLog.log("PhotonBackPose", poseEstimate.estimatedPose().toPose2d())
        );
        DogLog.log("PhotonColorHasTarget", PhotonUtil.Color.hasTargets());
        DogLog.log("PhotonBWTopRightHasTarget", PhotonUtil.BW.hasTargets(PhotonUtil.BW.BWCamera.TOP_RIGHT));
        DogLog.log("PhotonBWTopLeftHasTarget", PhotonUtil.BW.hasTargets(PhotonUtil.BW.BWCamera.TOP_LEFT));
        DogLog.log("PhotonBWBackHasTarget", PhotonUtil.BW.hasTargets(PhotonUtil.BW.BWCamera.BACK));
    }

    public void publishPose(StructPublisher<Pose2d> structPub, DoubleArrayPublisher arrayPub, StringPublisher prettyPub, Pose2d pose) {
        structPub.set(pose);
        arrayPub.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
        prettyPub.set("X: " + pose.getX() + ", Y: " + pose.getY() + ", Yaw: " + pose.getRotation().getDegrees());
    }
}
