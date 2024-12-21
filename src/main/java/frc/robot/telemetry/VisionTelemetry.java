package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LimelightUtil;

public class VisionTelemetry {
    private final Swerve swerve;
    private static final NetworkTable limelightTelemetryTable = Constants.NT_INSTANCE.getTable("LimelightTelemetry");
    private static final NetworkTable questNavTelemetryTable = Constants.NT_INSTANCE.getTable("questnav");

    private static final StringPublisher megaTagOnePub = limelightTelemetryTable.getStringTopic("MegaTag Pose").publish();
    private static final DoublePublisher nearestTagDistPub = limelightTelemetryTable.getDoubleTopic("Nearest Tag Distance").publish();
    private static final BooleanPublisher canAddMeasurementsPub = limelightTelemetryTable.getBooleanTopic("Can Add Measurements").publish();
    private static final StringPublisher poseEstimatePub = limelightTelemetryTable.getStringTopic("Robot Estimated Pose").publish();

    // TODO USE FOR ADSCOPE
    private static final IntegerSubscriber questFrameCountTopic = questNavTelemetryTable.getIntegerTopic("frameCount").subscribe(0);
    private static final DoubleSubscriber questTimestampTopic = questNavTelemetryTable.getDoubleTopic("timestamp").subscribe(0.0f);
    private static final DoubleSubscriber questBatteryTopic = questNavTelemetryTable.getDoubleTopic("batteryLevel").subscribe(0.0f);
    private static final FloatArraySubscriber questPositionTopic = questNavTelemetryTable.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private static final FloatArraySubscriber questQuaternionTopic = questNavTelemetryTable.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    private static final FloatArraySubscriber questEulerAnglesTopic = questNavTelemetryTable.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});

    private static final StringPublisher questPrettyPositionTopic = questNavTelemetryTable.getStringTopic("Quest Position").publish();
    private static final StringPublisher questPrettyRotationTopic = questNavTelemetryTable.getStringTopic("Quest Rotation").publish();

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

        canAddMeasurementsPub.set(LimelightUtil.tagExists() && LimelightUtil.getNearestTagDist() < 4.0);

        float[] questPositions = questPositionTopic.get();
        questPrettyPositionTopic.set("X: " + questPositions[2] + ", Y: " + questPositions[0] + ", Z: " + questPositions[1]);

        float[] questRotations = questEulerAnglesTopic.get();
        questPrettyRotationTopic.set("Pitch: " + questRotations[0] + ", Yaw: " + questRotations[1] + ", Roll: " + questRotations[2]);
    }

    public static double getQuestX() {
        return questPositionTopic.get()[2];
    }

    public static double getQuestY() {
        return questPositionTopic.get()[0];
    }

    public static double getQuestYaw() {
        return questEulerAnglesTopic.get()[1];
    }

    public static double getQuestTime() {
        return questTimestampTopic.get();
    }

}
