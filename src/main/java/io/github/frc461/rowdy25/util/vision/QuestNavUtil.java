package io.github.frc461.rowdy25.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import io.github.frc461.rowdy25.constants.Constants;

public final class QuestNavUtil {
    private static final NetworkTable QUESTNAV_NT = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.QuestNavConstants.QUESTNAV_NT_NAME);

    private static final IntegerSubscriber questMiso = QUESTNAV_NT.getIntegerTopic("miso").subscribe(0);
    private static final IntegerPublisher questMosi = QUESTNAV_NT.getIntegerTopic("mosi").publish();
    private static final DoubleArrayPublisher questResetPose = QUESTNAV_NT.getDoubleArrayTopic("resetpose").publish();

    private static final FloatArraySubscriber questPositionTopic = QUESTNAV_NT.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
    private static final FloatArraySubscriber questEulerAnglesTopic = QUESTNAV_NT.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});

    // Transformation applied to QuestNav pose to adjust origin to the pose estimator's origin
    public static final Transform2d robotToCameraOffset = new Transform2d(
            Constants.VisionConstants.QuestNavConstants.QUEST_FORWARD,
            Constants.VisionConstants.QuestNavConstants.QUEST_LEFT,
            new Rotation2d(Units.degreesToRadians(Constants.VisionConstants.QuestNavConstants.QUEST_YAW))
    );

    public static double getRawX() {
        return questPositionTopic.get()[2];
    }

    public static double getRawY() {
        return -questPositionTopic.get()[0];
    }

    public static double getRawZ() {
        return questPositionTopic.get()[1];
    }

    public static double stabilize(double angle) {
        return angle >= 180
                ? angle - ((int) ((angle - 180) / 360)) * 360 - 360
                : angle <= -180
                ? angle - ((int) ((angle + 180) / 360)) * 360 + 360
                : angle;
    }

    public static double getRawPitch() {
        return stabilize(questEulerAnglesTopic.get()[0]);
    }

    public static double getRawYaw() {
        return stabilize(-questEulerAnglesTopic.get()[1]);
    }

    public static double getRawRoll() {
        return stabilize(questEulerAnglesTopic.get()[2]);
    }

    public static Pose2d getCameraPose() {
        return new Pose2d(
                new Translation2d(getRawX(), getRawY()),
                new Rotation2d(Units.degreesToRadians(getRawYaw()))
        );
    }

    public static Pose2d getRobotPose() {
        return getCameraPose().plus(robotToCameraOffset.inverse());
    }

    public static void completeQuestPose() {
        if (questMiso.get() == 98 || questMiso.get() == 99) {
            questMosi.set(0);
        }
    }

    public static void setQuestPose(Pose2d robotPose) {
        Pose2d cameraPose = robotPose.plus(robotToCameraOffset);
        if (questMiso.get() != 98) {
            questResetPose.set(new double[]{
                    cameraPose.getX(),
                    cameraPose.getY(),
                    cameraPose.getRotation().getDegrees()
            });
            questMosi.set(2);
        }
    }
}
