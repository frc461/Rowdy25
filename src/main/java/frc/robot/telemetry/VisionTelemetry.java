package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Localizer;
import frc.robot.util.VisionUtil;

public class VisionTelemetry {
    private final Localizer localizer;

    public VisionTelemetry(Localizer localizer) {
        this.localizer = localizer;
    }

    private final NetworkTable visionTelemetryTable = Constants.NT_INSTANCE.getTable("VisionTelemetry");
    private final NetworkTable limelightTelemetryTable = Constants.NT_INSTANCE.getTable("LimelightTelemetry");
    private final NetworkTable photonTelemetryTable = Constants.NT_INSTANCE.getTable("PhotonTelemetry");
    private final NetworkTable questNavTelemetryTable = Constants.NT_INSTANCE.getTable("oculus");

    private final StringPublisher poseEstimatePub = visionTelemetryTable.getStringTopic("Robot Estimated Pose").publish();

    private final StringPublisher megaTagOnePub = limelightTelemetryTable.getStringTopic("MegaTagOne Pose").publish();
    private final StringPublisher megaTagTwoPub = limelightTelemetryTable.getStringTopic("MegaTagTwo Pose").publish();
    private final DoublePublisher nearestTagDistPub = limelightTelemetryTable.getDoubleTopic("Nearest Tag Distance").publish();
    private final BooleanPublisher canAddLLMeasurementsPub = limelightTelemetryTable.getBooleanTopic("Adding Limelight Measurements").publish();
    private final BooleanPublisher megaTagTwoCalibrated = limelightTelemetryTable.getBooleanTopic("MegaTagTwo Calibrated").publish();

    private final StringPublisher photonPosePub = photonTelemetryTable.getStringTopic("Photon Pose").publish();
    private final BooleanPublisher canAddPhotonMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Measurements").publish();

    private final StringPublisher questRawPose = questNavTelemetryTable.getStringTopic("Quest Position").publish();
    private final StringPublisher questRotationTopic = questNavTelemetryTable.getStringTopic("Quest Rotation").publish();
    private final StringPublisher questCorrectedPoseTopic = questNavTelemetryTable.getStringTopic("Quest Corrected Pose").publish();
    private final StringPublisher questOffsetTopic = questNavTelemetryTable.getStringTopic("Quest Offset").publish();
    private final BooleanPublisher questMode = questNavTelemetryTable.getBooleanTopic("Quest Mode").publish();

    public void publishValues() {
        Pose2d estimatedPose = localizer.getEstimatedPose();
        double estimateX = estimatedPose.getX();
        double estimateY = estimatedPose.getY();
        double estimateYaw = estimatedPose.getRotation().getDegrees();
        poseEstimatePub.set("X: " + estimateX + ", Y: " + estimateY + ", Yaw: " + estimateYaw);

        Pose2d megaTagOnePose = VisionUtil.Limelight.getMegaTagOnePose();
        double megaTagOneX = megaTagOnePose.getX();
        double megaTagOneY = megaTagOnePose.getY();
        double megaTagOneYaw = megaTagOnePose.getRotation().getDegrees();
        megaTagOnePub.set("X: " + megaTagOneX + ", Y: " + megaTagOneY + ", Yaw: " + megaTagOneYaw);

        Pose2d megaTagTwoPose = VisionUtil.Limelight.getMegaTagTwoPose();
        double megaTagTwoX = megaTagTwoPose.getX();
        double megaTagTwoY = megaTagTwoPose.getY();
        double megaTagTwoYaw = megaTagTwoPose.getRotation().getDegrees();
        megaTagTwoPub.set("X: " + megaTagTwoX + ", Y: " + megaTagTwoY + ", Yaw: " + megaTagTwoYaw);
        nearestTagDistPub.set(VisionUtil.Limelight.getNearestTagDist());
        canAddLLMeasurementsPub.set(VisionUtil.Limelight.isTagClear());
        megaTagTwoCalibrated.set(localizer.isMegaTagTwoConfigured());

        Pose2d photonPose = VisionUtil.Photon.BW.getPhotonPose();
        double photonPoseX = photonPose.getX();
        double photonPoseY = photonPose.getY();
        double photonPoseYaw = photonPose.getRotation().getDegrees();
        photonPosePub.set("X: " + photonPoseX + ", Y: " + photonPoseY + ", Yaw: " + photonPoseYaw);
        canAddPhotonMeasurementsPub.set(VisionUtil.Photon.BW.isTagClear());

        questRawPose.set("X: " + VisionUtil.QuestNav.getX() + ", Y: " + VisionUtil.QuestNav.getY() + ", Yaw: " + VisionUtil.QuestNav.getYaw());
        questRotationTopic.set("Pitch: " + VisionUtil.QuestNav.getPitch() + ", Yaw: " + VisionUtil.QuestNav.getYaw() + ", Roll: " + VisionUtil.QuestNav.getRoll());
        Pose2d questCorrectedPose = localizer.getQuestCorrectedPose();
        questCorrectedPoseTopic.set("X: " + questCorrectedPose.getX() + ", Y: " + questCorrectedPose.getY() + ", Yaw: " + questCorrectedPose.getRotation().getDegrees());
        Translation2d questTransOffset = localizer.getQuestTransOffset();
        Rotation2d questRotOffset = localizer.getQuestRotOffset();
        questOffsetTopic.set("X: " + questTransOffset.getX() + ", Y: " + questTransOffset.getY() + ", Yaw: " + questRotOffset.getDegrees());
        questMode.set(localizer.isQuestMode());

        logValues();
    }

    private void logValues() {
        Logger.recordOutput("LimelightMegaTagPose", VisionUtil.Limelight.getMegaTagOnePose());
        Logger.recordOutput("PoseEstimate", localizer.getEstimatedPose());
        Logger.recordOutput("QuestNavPose", localizer.getQuestCorrectedPose());
    }
}
