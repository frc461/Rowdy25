package frc.robot.telemetry;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.VisionUtil;

public class VisionTelemetry {
    private final Swerve swerve;
    private final NetworkTable limelightTelemetryTable = Constants.NT_INSTANCE.getTable("LimelightTelemetry");
    private final NetworkTable visionTelemetryTable = Constants.NT_INSTANCE.getTable("VisionTelemetry");
    private final NetworkTable questNavTelemetryTable = Constants.NT_INSTANCE.getTable("oculus");

    private final StringPublisher megaTagOnePub = limelightTelemetryTable.getStringTopic("MegaTag Pose").publish();
    private final DoublePublisher nearestTagDistPub = limelightTelemetryTable.getDoubleTopic("Nearest Tag Distance").publish();
    private final BooleanPublisher canAddMeasurementsPub = limelightTelemetryTable.getBooleanTopic("Adding Limelight Measurements").publish();

    private final StringPublisher poseEstimatePub = visionTelemetryTable.getStringTopic("Robot Estimated Pose").publish();

    private final StringPublisher questPrettyPositionTopic = questNavTelemetryTable.getStringTopic("Quest Position").publish();
    private final StringPublisher questPrettyRotationTopic = questNavTelemetryTable.getStringTopic("Quest Rotation").publish();

    public VisionTelemetry(Swerve swerve) {
        this.swerve = swerve;
    }

    public void publishValues() {
        Pose2d megaTagPose = VisionUtil.Limelight.getMegaTagOnePose();
        double megaTagX = megaTagPose.getTranslation().getX();
        double megaTagY = megaTagPose.getTranslation().getY();
        double megaTagYaw = megaTagPose.getRotation().getDegrees();
        megaTagOnePub.set("X: " + megaTagX + ", Y: " + megaTagY + ", Yaw: " + megaTagYaw);

        Pose2d estimatedPose = swerve.getEstimatedPose();
        double estimateX = estimatedPose.getTranslation().getX();
        double estimateY = estimatedPose.getTranslation().getY();
        double estimateYaw = estimatedPose.getRotation().getDegrees();
        poseEstimatePub.set("X: " + estimateX + ", Y: " + estimateY + ", Yaw: " + estimateYaw);

        nearestTagDistPub.set(VisionUtil.Limelight.getNearestTagDist());

        canAddMeasurementsPub.set(VisionUtil.Limelight.isTagClear());

        questPrettyPositionTopic.set("X: " + VisionUtil.Oculus.getX() + ", Y: " + VisionUtil.Oculus.getY() + ", Z: " + VisionUtil.Oculus.getZ());

        questPrettyRotationTopic.set("Pitch: " + VisionUtil.Oculus.getPitch() + ", Yaw: " + VisionUtil.Oculus.getYaw() + ", Roll: " + VisionUtil.Oculus.getRoll());

        logValues();
    }

    private void logValues() {
        Logger.recordOutput("LimelightMegaTagPose", VisionUtil.Limelight.getMegaTagOnePose());
        Logger.recordOutput("PoseEstimate", swerve.getEstimatedPose());
        Logger.recordOutput("QuestNavPose", VisionUtil.Oculus.getPose());
    }
}
