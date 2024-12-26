package frc.robot.telemetry;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Localizer;
import frc.robot.util.Elastic;
import frc.robot.util.VisionUtil;

import java.util.Arrays;
import java.util.EnumSet;

public class LocalizationTelemetry {
    private final Localizer localizer;

    public LocalizationTelemetry(Localizer localizer) {
        this.localizer = localizer;
    }

    private final NetworkTable localizationTelemetryTable = Constants.NT_INSTANCE.getTable("VisionTelemetry");
    private final NetworkTable limelightTelemetryTable = Constants.NT_INSTANCE.getTable("LimelightTelemetry");
    private final NetworkTable photonTelemetryTable = Constants.NT_INSTANCE.getTable("PhotonTelemetry");
    private final NetworkTable questNavTelemetryTable = Constants.NT_INSTANCE.getTable("oculus");

    private final StringPublisher poseEstimatePrettyPub = localizationTelemetryTable.getStringTopic("Estimated Pose").publish();
    private final StringPublisher questPosePrettyPub = localizationTelemetryTable.getStringTopic("Quest-Based Pose").publish();
    private final StringPublisher localizationStrategyPub = localizationTelemetryTable.getStringTopic("Localization Strategy").publish();

    private final StringPublisher megaTagOnePrettyPosePub = limelightTelemetryTable.getStringTopic("MegaTagOne Pose").publish();
    private final StringPublisher megaTagTwoPrettyPosePub = limelightTelemetryTable.getStringTopic("MegaTagTwo Pose").publish();
    private final DoublePublisher nearestTagDistPub = limelightTelemetryTable.getDoubleTopic("Nearest Tag Distance").publish();
    private final BooleanPublisher canAddLLMeasurementsPub = limelightTelemetryTable.getBooleanTopic("Adding Limelight Measurements").publish();
    private final BooleanPublisher megaTagTwoCalibratedPub = limelightTelemetryTable.getBooleanTopic("MegaTagTwo Calibrated").publish();

    private final StringPublisher photonPrettyPosePub = photonTelemetryTable.getStringTopic("Photon Pose").publish();
    private final BooleanPublisher canAddPhotonMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Measurements").publish();

    private final StringPublisher questRawPosePub = questNavTelemetryTable.getStringTopic("Quest Position").publish();
    private final StringPublisher questRotationPub = questNavTelemetryTable.getStringTopic("Quest Rotation").publish();
    private final StringPublisher questOffsetPub = questNavTelemetryTable.getStringTopic("Quest Offset").publish();
    private final DoubleSubscriber questBatterySub = questNavTelemetryTable.getDoubleTopic("batteryLevel").subscribe(0.0f);
    private final DoubleSubscriber questTimestampSub = questNavTelemetryTable.getDoubleTopic("timestamp").subscribe(0.0f);

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
    private final StructPublisher<Pose2d> photonPose2dPub = robotPoseTable.getStructTopic("Photon Pose2d", Pose2d.struct).publish();
    private final DoubleArrayPublisher photonPosePub = robotPoseTable.getDoubleArrayTopic("Photon Pose").publish();

    public void publishValues() {

        Pose2d questPose = localizer.getQuestPose();
        questPosePrettyPub.set("X: " + questPose.getX() + ", Y: " + questPose.getY() + ", Yaw: " + questPose.getRotation().getDegrees());
        Pose2d poseEstimate = localizer.getEstimatedPose();
        poseEstimatePrettyPub.set("X: " + poseEstimate.getX() + ", Y: " + poseEstimate.getY() + ", Yaw: " + poseEstimate.getRotation().getDegrees());
        localizationStrategyPub.set(localizer.getLocalizationStrategy());

        Pose2d megaTag1Pose = VisionUtil.Limelight.getMegaTagOnePose();
        megaTagOnePrettyPosePub.set("X: " + megaTag1Pose.getX() + ", Y: " + megaTag1Pose.getY() + ", Yaw: " + megaTag1Pose.getRotation().getDegrees());
        Pose2d megaTag2Pose = VisionUtil.Limelight.getMegaTagTwoPose();
        megaTagTwoPrettyPosePub.set("X: " + megaTag2Pose.getX() + ", Y: " + megaTag2Pose.getY() + ", Yaw: " + megaTag2Pose.getRotation().getDegrees());
        nearestTagDistPub.set(VisionUtil.Limelight.getNearestTagDist());
        canAddLLMeasurementsPub.set(VisionUtil.Limelight.isTagClear());
        megaTagTwoCalibratedPub.set(localizer.isMegaTagTwoConfigured());

        Pose2d photonPose = VisionUtil.Photon.BW.getPose();
        photonPrettyPosePub.set("X: " + photonPose.getX() + ", Y: " + photonPose.getY() + ", Yaw: " + photonPose.getRotation().getDegrees());
        canAddPhotonMeasurementsPub.set(VisionUtil.Photon.BW.hasTargets());

        questRawPosePub.set("X: " + VisionUtil.QuestNav.getRawX() + ", Y: " + VisionUtil.QuestNav.getRawY() + ", Yaw: " + VisionUtil.QuestNav.getRawYaw());
        questRotationPub.set("Pitch: " + VisionUtil.QuestNav.getRawPitch() + ", Yaw: " + VisionUtil.QuestNav.getRawYaw() + ", Roll: " + VisionUtil.QuestNav.getRawRoll());
        Transform2d questOffset = VisionUtil.QuestNav.questToFieldOffset;
        questOffsetPub.set("X: " + questOffset.getX() + ", Y: " + questOffset.getY() + ", Yaw: " + questOffset.getRotation().getDegrees());

        fieldTypePub.set("Field2d");
        questPose2dPub.set(questPose);
        questPosePub.set(new double[] {questPose.getX(), questPose.getY(), questPose.getRotation().getDegrees()});
        pose2dEstimatePub.set(poseEstimate);
        poseEstimatePub.set(new double[] {poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getRotation().getDegrees()});
        megaTagOnePose2dPub.set(megaTag1Pose);
        megaTagOnePosePub.set(new double[] {megaTag1Pose.getX(), megaTag1Pose.getY(), megaTag1Pose.getRotation().getDegrees()});
        megaTagTwoPose2dPub.set(megaTag2Pose);
        megaTagTwoPosePub.set(new double[] {megaTag2Pose.getX(), megaTag2Pose.getY(), megaTag2Pose.getRotation().getDegrees()});
        photonPose2dPub.set(photonPose);
        photonPosePub.set(new double[] {photonPose.getX(), photonPose.getY(), photonPose.getRotation().getDegrees()});

        logValues();
    }

    private void logValues() {
        DogLog.log("PoseEstimate", localizer.getEstimatedPose());
        DogLog.log("QuestNavPose", localizer.getQuestPose());
        DogLog.log("LocalizationStrategy", localizer.getLocalizationStrategy());
        DogLog.log("LimelightMegaTagPose", VisionUtil.Limelight.getMegaTagOnePose());
        DogLog.log("LimelightMegaTagTwoPose", VisionUtil.Limelight.getMegaTagTwoPose());
        DogLog.log("LimelightHasTarget", VisionUtil.Limelight.tagExists());
        DogLog.log("MegaTagTwoCalibrated", localizer.isMegaTagTwoConfigured());
        DogLog.log("PhotonPose", VisionUtil.Photon.BW.getPose());
        DogLog.log("PhotonColorHasTarget", VisionUtil.Photon.Color.hasTargets());
        DogLog.log("PhotonBWHasTarget", VisionUtil.Photon.BW.hasTargets());
    }

    public void registerListeners() {
        Constants.NT_INSTANCE.addListener(questBatterySub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), (event) -> {
                if (Arrays.stream(questBatterySub.readQueueValues()).noneMatch(x -> x <= 0.005)
                            && questBatterySub.get() <= 0.005) {
                        DogLog.logFault(Constants.Logger.QuestFault.QUEST_DIED);
                        Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "Quest Nav", "Quest ran out of battery! Press B to switch to PoseEstimator."));
                }
                if (Arrays.stream(questBatterySub.readQueueValues()).noneMatch(x -> x <= 0.1)
                            && questBatterySub.get() <= 0.1) {
                        DogLog.logFault(Constants.Logger.QuestFault.QUEST_LOW_BATTERY);
                        Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.WARNING, "Quest Nav", "Quest has less than 10% battery left! Current Percent: " + (int) (questBatterySub.get() * 100)));
                }
        });
    }
}
