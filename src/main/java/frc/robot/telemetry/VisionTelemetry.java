package frc.robot.telemetry;

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

    private final StringPublisher megaTagOnePub = limelightTelemetryTable.getStringTopic("MegaTag Pose").publish();
    private final DoublePublisher nearestTagDistPub = limelightTelemetryTable.getDoubleTopic("Nearest Tag Distance").publish();
    private final BooleanPublisher canAddMeasurementsPub = limelightTelemetryTable.getBooleanTopic("Adding Limelight Measurements").publish();

    private final StringPublisher photonPosePub = photonTelemetryTable.getStringTopic("Photon Pose").publish();

    private final StringPublisher questPositionTopic = questNavTelemetryTable.getStringTopic("Quest Position").publish();
    private final StringPublisher questRotationTopic = questNavTelemetryTable.getStringTopic("Quest Rotation").publish();

    public void publishValues() {
        Pose2d estimatedPose = localizer.getEstimatedPose();
        double estimateX = estimatedPose.getX();
        double estimateY = estimatedPose.getY();
        double estimateYaw = estimatedPose.getRotation().getDegrees();
        poseEstimatePub.set("X: " + estimateX + ", Y: " + estimateY + ", Yaw: " + estimateYaw);

        Pose2d megaTagPose = VisionUtil.Limelight.getMegaTagOnePose();
        double megaTagX = megaTagPose.getX();
        double megaTagY = megaTagPose.getY();
        double megaTagYaw = megaTagPose.getRotation().getDegrees();
        megaTagOnePub.set("X: " + megaTagX + ", Y: " + megaTagY + ", Yaw: " + megaTagYaw);
        nearestTagDistPub.set(VisionUtil.Limelight.getNearestTagDist());
        canAddMeasurementsPub.set(VisionUtil.Limelight.isTagClear());

        Pose2d photonPose = VisionUtil.Photon.BW.getPhotonPose();
        double photonPoseX = photonPose.getX();
        double photonPoseY = photonPose.getY();
        double photonPoseYaw = photonPose.getRotation().getDegrees();
        photonPosePub.set("X: " + photonPoseX + ", Y: " + photonPoseY + ", Yaw: " + photonPoseYaw);

        questPositionTopic.set("X: " + VisionUtil.QuestNav.getX() + ", Y: " + VisionUtil.QuestNav.getY() + ", Z: " + VisionUtil.QuestNav.getZ());
        questRotationTopic.set("Pitch: " + VisionUtil.QuestNav.getPitch() + ", Yaw: " + VisionUtil.QuestNav.getYaw() + ", Roll: " + VisionUtil.QuestNav.getRoll());
    }
}
