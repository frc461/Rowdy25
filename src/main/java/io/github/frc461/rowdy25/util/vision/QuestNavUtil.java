package io.github.frc461.rowdy25.util.vision;

/*
 * Copyright (C) 2025-present 461 Boosters FIRST, Inc. dba Westside Robotics - The Rowdy 25.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import io.github.frc461.rowdy25.constants.Constants;

/**
 * Utility class for interfacing with QuestNav, an application on the Oculus Quest 3s for localization and navigation. This class provides methods to retrieve and process position and orientation data from the QuestNav NetworkTable, as well as configure pose resets.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public final class QuestNavUtil {
    /** The QuestNav network table. */
    private static final NetworkTable QUESTNAV_NT = Constants.NT_INSTANCE.getTable(Constants.VisionConstants.QuestNavConstants.QUESTNAV_NT_NAME);

    /** NetworkTable subscriber for the QuestNav state or status (MISO). */
    private static final IntegerSubscriber questMiso = QUESTNAV_NT.getIntegerTopic("miso").subscribe(0);
    /** NetworkTable publisher for sending commands to QuestNav (MOSI). */
    private static final IntegerPublisher questMosi = QUESTNAV_NT.getIntegerTopic("mosi").publish();
    /** NetworkTable publisher for sending pose reset data to QuestNav. */
    private static final DoubleArrayPublisher questResetPose = QUESTNAV_NT.getDoubleArrayTopic("resetpose").publish();

    /** NetworkTable subscriber for the raw position data array from QuestNav. */
    private static final FloatArraySubscriber questPositionTopic = QUESTNAV_NT.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
    /** NetworkTable subscriber for the raw Euler angles data array from QuestNav. */
    private static final FloatArraySubscriber questEulerAnglesTopic = QUESTNAV_NT.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});

    /** Transformation applied to QuestNav pose to adjust origin to the pose estimator's origin. */
    public static final Transform2d robotToCameraOffset = new Transform2d(
            Constants.VisionConstants.QuestNavConstants.QUEST_FORWARD,
            Constants.VisionConstants.QuestNavConstants.QUEST_LEFT,
            new Rotation2d(Units.degreesToRadians(Constants.VisionConstants.QuestNavConstants.QUEST_YAW))
    );

    /**
     * Retrieves the raw X position from the QuestNav network table.
     *
     * @return The raw X distance in meters.
     */
    public static double getRawX() {
        return questPositionTopic.get()[2];
    }

    /**
     * Retrieves the raw Y position from the QuestNav network table.
     *
     * @return The raw Y distance in meters.
     */
    public static double getRawY() {
        return -questPositionTopic.get()[0];
    }

    /**
     * Retrieves the raw Z position from the QuestNav network table.
     *
     * @return The raw Z distance in meters.
     */
    public static double getRawZ() {
        return questPositionTopic.get()[1];
    }

    /**
     * Stabilizes an angle to ensure it falls within the nominal [-180, 180] degree range.
     *
     * @param angle The angle to wrap/stabilize, in degrees.
     * @return The wrapped angle in degrees.
     */
    public static double stabilize(double angle) {
        return angle >= 180
                ? angle - ((int) ((angle - 180) / 360)) * 360 - 360
                : angle <= -180
                ? angle - ((int) ((angle + 180) / 360)) * 360 + 360
                : angle;
    }

    /**
     * Retrieves the raw pitch angle from the QuestNav network table.
     *
     * @return The raw pitch angle in degrees, stabilized to [-180, 180].
     */
    public static double getRawPitch() {
        return stabilize(questEulerAnglesTopic.get()[0]);
    }

    /**
     * Retrieves the raw yaw angle from the QuestNav network table.
     *
     * @return The raw yaw angle in degrees, stabilized to [-180, 180].
     */
    public static double getRawYaw() {
        return stabilize(-questEulerAnglesTopic.get()[1]);
    }

    /**
     * Retrieves the raw roll angle from the QuestNav network table.
     *
     * @return The raw roll angle in degrees, stabilized to [-180, 180].
     */
    public static double getRawRoll() {
        return stabilize(questEulerAnglesTopic.get()[2]);
    }

    /**
     * Constructs the pose of the QuestNav camera on the field using the raw X, Y, and stabilized yaw data.
     *
     * @return A {@link Pose2d} representing the overall camera pose.
     */
    public static Pose2d getCameraPose() {
        return new Pose2d(
                new Translation2d(getRawX(), getRawY()),
                new Rotation2d(Units.degreesToRadians(getRawYaw()))
        );
    }

    /**
     * Computes the robot's pose on the field based on the QuestNav camera's pose and its offset from the robot center.
     *
     * @return A {@link Pose2d} representing the localized robot pose.
     */
    public static Pose2d getRobotPose() {
        return getCameraPose().plus(robotToCameraOffset.inverse());
    }

    /**
     * Clears the active initialization state of the QuestNav application if currently processing a reset phase.
     */
    public static void completeQuestPose() {
        if (questMiso.get() == 98 || questMiso.get() == 99) {
            questMosi.set(0);
        }
    }

    /**
     * Publishes a requested reset pose to the QuestNav application. The robot-to-camera offset is first applied to the specified robot pose before that calculated pose is passed through to the network table as the actual reset pose.
     *
     * @param robotPose The definitive robot pose to initialize or reset the QuestNav to.
     */
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
