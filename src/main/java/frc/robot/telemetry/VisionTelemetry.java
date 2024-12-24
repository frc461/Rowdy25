package frc.robot.telemetry;

import dev.doglog.DogLog;
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
    private final StructPublisher<Pose2d> fusedPosePub = visionTelemetryTable.getStructTopic("Real Fused Pose", Pose2d.struct).publish();

    private final StringPublisher megaTagOnePub = limelightTelemetryTable.getStringTopic("MegaTagOne Pose").publish();
    private final StringPublisher megaTagTwoPub = limelightTelemetryTable.getStringTopic("MegaTagTwo Pose").publish();
    private final DoublePublisher nearestTagDistPub = limelightTelemetryTable.getDoubleTopic("Nearest Tag Distance").publish();
    private final BooleanPublisher canAddLLMeasurementsPub = limelightTelemetryTable.getBooleanTopic("Adding Limelight Measurements").publish();
    private final BooleanPublisher megaTagTwoCalibrated = limelightTelemetryTable.getBooleanTopic("MegaTagTwo Calibrated").publish();

    private final StringPublisher photonPosePub = photonTelemetryTable.getStringTopic("Photon Pose").publish();
    private final StructPublisher<Pose2d> cameraPosePub = photonTelemetryTable.getStructTopic("Real Photon Pose", Pose2d.struct).publish();

    private final BooleanPublisher canAddPhotonMeasurementsPub = photonTelemetryTable.getBooleanTopic("Adding Photon Measurements").publish();

    private final StringPublisher questRawPose = questNavTelemetryTable.getStringTopic("Quest Position").publish();
    private final StringPublisher questRotationTopic = questNavTelemetryTable.getStringTopic("Quest Rotation").publish();
    private final StringPublisher questCorrectedPoseTopic = questNavTelemetryTable.getStringTopic("Quest Corrected Pose").publish();
    private final StringPublisher questOffsetTopic = questNavTelemetryTable.getStringTopic("Quest Offset").publish();
    private final BooleanPublisher questMode = questNavTelemetryTable.getBooleanTopic("Quest Mode").publish();
    private final StructPublisher<Pose2d> questPosePub = questNavTelemetryTable.getStructTopic("Real Quest Pose", Pose2d.struct).publish();


    public void publishValues() {
        Pose2d estimatedPose = localizer.getEstimatedPose();
        double estimateX = estimatedPose.getX();
        double estimateY = estimatedPose.getY();
        double estimateYaw = estimatedPose.getRotation().getDegrees();
        poseEstimatePub.set("X: " + estimateX + ", Y: " + estimateY + ", Yaw: " + estimateYaw);
        fusedPosePub.set(estimatedPose);

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

        Pose2d photonPose = VisionUtil.Photon.BW.getPose();
        double photonPoseX = photonPose.getX();
        double photonPoseY = photonPose.getY();
        double photonPoseYaw = photonPose.getRotation().getDegrees();
        photonPosePub.set("X: " + photonPoseX + ", Y: " + photonPoseY + ", Yaw: " + photonPoseYaw);
        cameraPosePub.set(photonPose);
        canAddPhotonMeasurementsPub.set(VisionUtil.Photon.BW.isTagClear());

        questRawPose.set("X: " + VisionUtil.QuestNav.getRawX() + ", Y: " + VisionUtil.QuestNav.getRawY() + ", Yaw: " + VisionUtil.QuestNav.getRawYaw());
        questRotationTopic.set("Pitch: " + VisionUtil.QuestNav.getRawPitch() + ", Yaw: " + VisionUtil.QuestNav.getRawYaw() + ", Roll: " + VisionUtil.QuestNav.getRawRoll());
        Pose2d questCorrectedPose = localizer.getQuestPose();
        questCorrectedPoseTopic.set("X: " + questCorrectedPose.getX() + ", Y: " + questCorrectedPose.getY() + ", Yaw: " + questCorrectedPose.getRotation().getDegrees());
        questMode.set(localizer.isQuestMode());
        questPosePub.set(questCorrectedPose);

        logValues();
    }

    private void logValues() {
        DogLog.log("PoseEstimate", localizer.getEstimatedPose());
        DogLog.log("LimelightMegaTagPose", VisionUtil.Limelight.getMegaTagOnePose());
        DogLog.log("LimelightMegaTagTwoPose", VisionUtil.Limelight.getMegaTagTwoPose());
        DogLog.log("PhotonPose", VisionUtil.Photon.BW.getPose());
        DogLog.log("QuestNavPose", localizer.getQuestPose());
        DogLog.log("QuestMode", localizer.isQuestMode());

        DogLog.log("PhotonColorHasTarget", VisionUtil.Photon.Color.hasTargets());
        DogLog.log("PhotonBWHasTarget", VisionUtil.Photon.BW.hasTargets());
        DogLog.log("LimelightHasTarget", VisionUtil.Limelight.tagExists());
        DogLog.log("MegaTagTwoActive", localizer.isMegaTagTwoConfigured());

        // Log Quest faults
        if (!(VisionUtil.QuestNav.isQuestAlive()) || VisionUtil.QuestNav.getBatteryLevel() == 461) { 
            DogLog.logFault(Constants.Logger.RobotFault.QUEST_DISCONNECT); 
        }
        if (VisionUtil.QuestNav.getBatteryLevel() <= 0.005) {
            DogLog.logFault(Constants.Logger.RobotFault.QUEST_DIED);
        } else if (VisionUtil.QuestNav.getBatteryLevel() <= 0.1) {
            DogLog.logFault(Constants.Logger.RobotFault.QUEST_LOW_BATTERY);
        }
    }
}
