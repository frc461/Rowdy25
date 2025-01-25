package frc.robot.subsystems.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.util.Elastic;
import frc.robot.util.VisionUtil;

import java.util.EnumSet;

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

    // TODO CONSISTENCY, CONSTANTIZE ALL TABLE STRING NAMES
    private final NetworkTable localizationTelemetryTable = Constants.NT_INSTANCE.getTable("LocalizationTelemetry");
    private final NetworkTable limelightTelemetryTable = Constants.NT_INSTANCE.getTable("LimelightTelemetry");
    private final NetworkTable photonTelemetryTable = Constants.NT_INSTANCE.getTable("PhotonTelemetry");
    private final NetworkTable questNavTelemetryTable = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.QuestNavConstants.QUESTNAV_NT_NAME);

    private final StringPublisher poseEstimatePrettyPub = localizationTelemetryTable.getStringTopic("Estimated Pose").publish();
    private final StringPublisher questPosePrettyPub = localizationTelemetryTable.getStringTopic("Quest-Based Pose").publish();
    private final StringPublisher localizationStrategyPub = localizationTelemetryTable.getStringTopic("Localization Strategy").publish();

    private final StringPublisher megaTagOnePosePrettyPub = limelightTelemetryTable.getStringTopic("MegaTagOne Pose").publish();
    private final StringPublisher megaTagTwoPosePrettyPub = limelightTelemetryTable.getStringTopic("MegaTagTwo Pose").publish();
    private final DoublePublisher nearestTagDistPub = limelightTelemetryTable.getDoubleTopic("Nearest Tag Distance").publish();
    private final BooleanPublisher canAddLLMeasurementsPub = limelightTelemetryTable.getBooleanTopic("Adding Limelight Measurements").publish();

    private final StringPublisher photonTopRightPosePrettyPub = photonTelemetryTable.getStringTopic("Photon Top Right Pose").publish();
    private final StringPublisher photonTopLeftPosePrettyPub = photonTelemetryTable.getStringTopic("Photon Top Left Pose").publish();
    private final StringPublisher photonBackPosePrettyPub = photonTelemetryTable.getStringTopic("Photon Back Pose").publish();
    private final BooleanPublisher canAddTopRightMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Measurements").publish();
    private final BooleanPublisher canAddTopLeftMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Measurements").publish();
    private final BooleanPublisher canAddBackMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Measurements").publish();

    private final StringPublisher questCameraPosePrettyPub = questNavTelemetryTable.getStringTopic("Quest-Based Camera Pose").publish();
    private final StringPublisher questRotationPub = questNavTelemetryTable.getStringTopic("Quest Rotation").publish();
    private final BooleanPublisher questHasCalibratedOnceWhenNear = questNavTelemetryTable.getBooleanTopic("Quest Has Calibrated When Near").publish();
    private final DoubleSubscriber questBatterySub = questNavTelemetryTable.getDoubleTopic("batteryPerent").subscribe(0.0f);
    private final DoubleSubscriber questTimestampSub = questNavTelemetryTable.getDoubleTopic("timestamp").subscribe(0.0f);

    private boolean questSendDisconnectMessage = true;
    private boolean questSendDiedMessage = true;
    private boolean questSendBatteryLowMessage = true;

    private final NetworkTable robotPoseTable = Constants.NT_INSTANCE.getTable("Pose");
    private final StringPublisher fieldTypePub = robotPoseTable.getStringTopic(".type").publish();
    private final StructPublisher<Pose2d> pose2dEstimatePub = robotPoseTable.getStructTopic("Estimated Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher poseEstimatePub = robotPoseTable.getDoubleArrayTopic("Estimated Pose").publish();
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
    private final StructPublisher<Pose2d> questCameraPose2dPub = robotPoseTable.getStructTopic("Quest-Based Camera Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher questCameraPosePub = robotPoseTable.getDoubleArrayTopic("Quest-Based Camera Pose").publish();

    public void publishValues() {
        localizationStrategyPub.set(localizer.getLocalizationStrategy());

        nearestTagDistPub.set(VisionUtil.Limelight.getNearestTagDist());
        canAddLLMeasurementsPub.set(VisionUtil.Limelight.isTagClear());

        canAddTopRightMeasurementsPub.set(VisionUtil.Photon.BW.isTagClear(VisionUtil.Photon.BW.BWCamera.TOP_RIGHT));
        canAddTopLeftMeasurementsPub.set(VisionUtil.Photon.BW.isTagClear(VisionUtil.Photon.BW.BWCamera.TOP_LEFT));
        canAddBackMeasurementsPub.set(VisionUtil.Photon.BW.isTagClear(VisionUtil.Photon.BW.BWCamera.BACK));

        questRotationPub.set("Roll: " + VisionUtil.QuestNav.getRawRoll() + ", Pitch: " + VisionUtil.QuestNav.getRawPitch() + ", Yaw: " + VisionUtil.QuestNav.getRawYaw());
        questHasCalibratedOnceWhenNear.set(localizer.hasCalibratedOnceWhenNear());

        fieldTypePub.set("Field2d");
        publishPose(pose2dEstimatePub, poseEstimatePub, poseEstimatePrettyPub, localizer.getEstimatedPose());
        publishPose(questPose2dPub, questPosePub, questPosePrettyPub, localizer.getQuestPose());
        publishPose(megaTagOnePose2dPub, megaTagOnePosePub, megaTagOnePosePrettyPub, VisionUtil.Limelight.getMegaTagOnePose());
        publishPose(megaTagTwoPose2dPub, megaTagTwoPosePub, megaTagTwoPosePrettyPub, VisionUtil.Limelight.getMegaTagTwoPose());
        publishPose(photonTopRightPose2dPub, photonTopRightPosePub, photonTopRightPosePrettyPub,
                localizer.getUpdatedPhotonPoseEstimate(VisionUtil.Photon.BW.BWCamera.TOP_RIGHT).estimatedPose().toPose2d());
        publishPose(photonTopLeftPose2dPub, photonTopLeftPosePub, photonTopLeftPosePrettyPub,
                localizer.getUpdatedPhotonPoseEstimate(VisionUtil.Photon.BW.BWCamera.TOP_LEFT).estimatedPose().toPose2d());
        publishPose(photonBackPose2dPub, photonBackPosePub, photonBackPosePrettyPub,
                localizer.getUpdatedPhotonPoseEstimate(VisionUtil.Photon.BW.BWCamera.BACK).estimatedPose().toPose2d());
        publishPose(questCameraPose2dPub, questCameraPosePub, questCameraPosePrettyPub, VisionUtil.QuestNav.getCameraPose());

        logValues();
    }

    private void logValues() {
        DogLog.log("PoseEstimate", localizer.getEstimatedPose());
        DogLog.log("QuestNavPose", localizer.getQuestPose());
        DogLog.log("LocalizationStrategy", localizer.getLocalizationStrategy());
        DogLog.log("LimelightMegaTagOnePose", VisionUtil.Limelight.getMegaTagOnePose());
        DogLog.log("LimelightMegaTagTwoPose", VisionUtil.Limelight.getMegaTagTwoPose());
        DogLog.log("LimelightHasTarget", VisionUtil.Limelight.tagExists());
        DogLog.log("PhotonTopRightPose", VisionUtil.Photon.BW.getMultiTagPose(VisionUtil.Photon.BW.BWCamera.TOP_RIGHT).estimatedPose().toPose2d());
        DogLog.log("PhotonTopLeftPose", VisionUtil.Photon.BW.getMultiTagPose(VisionUtil.Photon.BW.BWCamera.TOP_LEFT).estimatedPose().toPose2d());
        DogLog.log("PhotonBackPose", VisionUtil.Photon.BW.getMultiTagPose(VisionUtil.Photon.BW.BWCamera.BACK).estimatedPose().toPose2d());
        DogLog.log("PhotonColorHasTarget", VisionUtil.Photon.Color.hasTargets());
        DogLog.log("PhotonBWTopRightHasTarget", VisionUtil.Photon.BW.hasTargets(VisionUtil.Photon.BW.BWCamera.TOP_RIGHT));
        DogLog.log("PhotonBWTopLeftHasTarget", VisionUtil.Photon.BW.hasTargets(VisionUtil.Photon.BW.BWCamera.TOP_LEFT));
        DogLog.log("PhotonBWBackHasTarget", VisionUtil.Photon.BW.hasTargets(VisionUtil.Photon.BW.BWCamera.BACK));

        if (DriverStation.isEnabled() && questTimestampSub.getLastChange() <= (Timer.getTimestamp() - 2) * Constants.ONE_MILLION && questSendDisconnectMessage) {
            DogLog.logFault(QuestFault.QUEST_DISCONNECTED);
            Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "Quest Nav", "Quest has been disconnected! Press B to switch to PoseEstimator.", 7000));
            questSendDisconnectMessage = false;
        }
    }

    public void registerListeners() {
        Constants.NT_INSTANCE.addListener(questBatterySub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), (event) -> {
                if (questBatterySub.get() <= 0.5 && questSendDiedMessage) {
                        DogLog.logFault(QuestFault.QUEST_DIED);
                        Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "Quest Nav", "Quest ran out of battery! Press B to switch to PoseEstimator.", 7000));
                        questSendDiedMessage = false;
                }
                if (questBatterySub.get() <= 10 && questSendBatteryLowMessage) {
                        DogLog.logFault(QuestFault.QUEST_LOW_BATTERY);
                        Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "Quest Nav", "Quest has less than 10% battery left! Current Percent: " + questBatterySub.get(), 7000));
                        questSendBatteryLowMessage = false;
                }
        });
    }

    public void publishPose(StructPublisher<Pose2d> structPub, DoubleArrayPublisher arrayPub, StringPublisher prettyPub, Pose2d pose) {
        structPub.set(pose);
        arrayPub.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
        prettyPub.set("X: " + pose.getX() + ", Y: " + pose.getY() + ", Yaw: " + pose.getRotation().getDegrees());
    }
}
