package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.Constants;
import frc.robot.util.LimelightUtil;

public class LimelightTelemetry {
    private final NetworkTable limelightTelemetryTable = Constants.NT_INSTANCE.getTable("LimelightTelemetry");
    private final StringPublisher megaTagOnePublisher = limelightTelemetryTable.getStringTopic("MegaTag Pose").publish();

    public void publishValues() {
        Pose2d megaTagPose = LimelightUtil.getMegaTagOnePose();
        double megaTagX = megaTagPose.getTranslation().getX();
        double megaTagY = megaTagPose.getTranslation().getY();
        double megaTagYaw = megaTagPose.getRotation().getDegrees();
        megaTagOnePublisher.set("X: " + megaTagX + ", Y: " + megaTagY + ", Yaw: " + megaTagYaw);
    }
}
