package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LimelightUtil;

public class VisionTelemetry {
    private final Swerve swerve;
    private final NetworkTable limelightTelemetryTable = Constants.NT_INSTANCE.getTable("LimelightTelemetry");
    private final NetworkTable questNavTelemetryTable = Constants.NT_INSTANCE.getTable("oculus");

    private final StringPublisher megaTagOnePub = limelightTelemetryTable.getStringTopic("MegaTag Pose").publish();
    private final DoublePublisher nearestTagDistPub = limelightTelemetryTable.getDoubleTopic("Nearest Tag Distance").publish();
    private final BooleanPublisher canAddMeasurementsPub = limelightTelemetryTable.getBooleanTopic("Can Add Measurements").publish();
    private final StringPublisher poseEstimatePub = limelightTelemetryTable.getStringTopic("Robot Estimated Pose").publish();

    // TODO USE FOR ADSCOPE
    private final IntegerSubscriber questFrameCountTopic = questNavTelemetryTable.getIntegerTopic("frameCount").subscribe(0);
    private final DoubleSubscriber questTimestampTopic = questNavTelemetryTable.getDoubleTopic("timestamp").subscribe(0.0f);
    private final DoubleSubscriber questBatteryTopic = questNavTelemetryTable.getDoubleTopic("batteryLevel").subscribe(0.0f);
    private final FloatArraySubscriber questPositionTopic = questNavTelemetryTable.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private final FloatArraySubscriber questQuaternionTopic = questNavTelemetryTable.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    private final FloatArraySubscriber questEulerAnglesTopic = questNavTelemetryTable.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});

    private final StringPublisher questPrettyPositionTopic = questNavTelemetryTable.getStringTopic("Quest Position").publish();
    private final StringPublisher questPrettyRotationTopic = questNavTelemetryTable.getStringTopic("Quest Rotation").publish();

    public VisionTelemetry(Swerve swerve) {
        this.swerve = swerve;
    }

    public void publishValues() {
        Pose2d megaTagPose = LimelightUtil.getMegaTagOnePose();
        double megaTagX = megaTagPose.getTranslation().getX();
        double megaTagY = megaTagPose.getTranslation().getY();
        double megaTagYaw = megaTagPose.getRotation().getDegrees();
        megaTagOnePub.set("X: " + megaTagX + ", Y: " + megaTagY + ", Yaw: " + megaTagYaw);

        Pose2d estimatedPose = swerve.getEstimatedPose();
        double estimateX = estimatedPose.getTranslation().getX();
        double estimateY = estimatedPose.getTranslation().getY();
        double estimateYaw = estimatedPose.getRotation().getDegrees();
        poseEstimatePub.set("X: " + estimateX + ", Y: " + estimateY + ", Yaw: " + estimateYaw);

        nearestTagDistPub.set(LimelightUtil.getNearestTagDist());

        canAddMeasurementsPub.set(LimelightUtil.tagExists() && LimelightUtil.getNearestTagDist() < 2.0);

        float[] questPositions = questPositionTopic.get();
        questPrettyPositionTopic.set("X: " + questPositions[2] + ", Y: " + questPositions[0] + ", Z: " + questPositions[1]);

        float[] questRotations = questEulerAnglesTopic.get();
        questPrettyRotationTopic.set("Pitch: " + questRotations[0] + ", Yaw: " + questRotations[1] + ", Roll: " + questRotations[2]);
    }
}
