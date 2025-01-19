package frc.robot.constants;

import frc.robot.constants.variants.DefaultConstants;
import frc.robot.constants.variants.SimConstants;

public enum RobotIdentity {
    ROWDY,
    SIM;

    private static RobotIdentity getIdentity() {
        String mac = MacAddress.getMACAddress();
        if (!mac.isEmpty()) {
            if (mac.equals(MacAddress.ROWDY)) {
                return ROWDY;
            }
        }
        return SIM;
    }

    public static void initializeConstants() {
        setDefaultConstants();
        if (getIdentity() == RobotIdentity.SIM) {
            setSimConstants();
        }
    }

    private static void setDefaultConstants() {
        Constants.CAN_BUS = DefaultConstants.CAN_BUS;
        Constants.BLUE_DEFAULT_ROTATION = DefaultConstants.BLUE_DEFAULT_ROTATION;
        Constants.RED_DEFAULT_ROTATION = DefaultConstants.RED_DEFAULT_ROTATION;
        Constants.ALLIANCE_SUPPLIER = DefaultConstants.ALLIANCE_SUPPLIER;
        Constants.MAX_VEL = DefaultConstants.MAX_VEL;
        Constants.MAX_REAL_ANGULAR_VEL = DefaultConstants.MAX_REAL_ANGULAR_VEL;
        Constants.MAX_CONTROLLED_ANGULAR_VEL = DefaultConstants.MAX_CONTROLLED_ANGULAR_VEL;
        Constants.MAX_ACCEL = DefaultConstants.MAX_ACCEL;
        Constants.MAX_CONTROLLED_ACCEL = DefaultConstants.MAX_CONTROLLED_ACCEL;
        Constants.MAX_ANGULAR_ACCEL = DefaultConstants.MAX_ANGULAR_ACCEL;
        Constants.NT_INSTANCE = DefaultConstants.NT_INSTANCE;
        Constants.ONE_MILLION = DefaultConstants.ONE_MILLION;
        Constants.STICK_DEADBAND = DefaultConstants.STICK_DEADBAND;

        Constants.AutoConstants.ROBOT_CONFIG = DefaultConstants.AutoConstants.ROBOT_CONFIG;
        Constants.AutoConstants.NOTE_CHECK_MARKER = DefaultConstants.AutoConstants.NOTE_CHECK_MARKER;
        Constants.AutoConstants.PATH_CONSTRAINTS = DefaultConstants.AutoConstants.PATH_CONSTRAINTS;
        Constants.AutoConstants.NOTE_SEARCH_DEGREE_SLANT = DefaultConstants.AutoConstants.NOTE_SEARCH_DEGREE_SLANT;
        Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT = DefaultConstants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT;
        Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT = DefaultConstants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;

        Constants.VisionConstants.ODOM_STD_DEV = DefaultConstants.VisionConstants.ODOM_STD_DEV;
        Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION = DefaultConstants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION;
        Constants.VisionConstants.VISION_STD_DEV_FUNCTION = DefaultConstants.VisionConstants.VISION_STD_DEV_FUNCTION;
        Constants.VisionConstants.LimelightConstants.LIMELIGHT_NT_NAME = DefaultConstants.VisionConstants.LimelightConstants.LIMELIGHT_NT_NAME;
        Constants.VisionConstants.LimelightConstants.LL_FORWARD = DefaultConstants.VisionConstants.LimelightConstants.LL_FORWARD;
        Constants.VisionConstants.LimelightConstants.LL_RIGHT = DefaultConstants.VisionConstants.LimelightConstants.LL_RIGHT;
        Constants.VisionConstants.LimelightConstants.LL_UP = DefaultConstants.VisionConstants.LimelightConstants.LL_UP;
        Constants.VisionConstants.LimelightConstants.LL_ROLL = DefaultConstants.VisionConstants.LimelightConstants.LL_ROLL;
        Constants.VisionConstants.LimelightConstants.LL_PITCH = DefaultConstants.VisionConstants.LimelightConstants.LL_PITCH;
        Constants.VisionConstants.LimelightConstants.LL_YAW = DefaultConstants.VisionConstants.LimelightConstants.LL_YAW;
        Constants.VisionConstants.LimelightConstants.LL_MAX_TAG_CLEAR_DIST = DefaultConstants.VisionConstants.LimelightConstants.LL_MAX_TAG_CLEAR_DIST;
        Constants.VisionConstants.PhotonConstants.BW_FORWARD = DefaultConstants.VisionConstants.PhotonConstants.BW_FORWARD;
        Constants.VisionConstants.PhotonConstants.BW_LEFT = DefaultConstants.VisionConstants.PhotonConstants.BW_LEFT;
        Constants.VisionConstants.PhotonConstants.BW_UP = DefaultConstants.VisionConstants.PhotonConstants.BW_UP;
        Constants.VisionConstants.PhotonConstants.BW_ROLL = DefaultConstants.VisionConstants.PhotonConstants.BW_ROLL;
        Constants.VisionConstants.PhotonConstants.BW_PITCH = DefaultConstants.VisionConstants.PhotonConstants.BW_PITCH;
        Constants.VisionConstants.PhotonConstants.BW_YAW = DefaultConstants.VisionConstants.PhotonConstants.BW_YAW;
        Constants.VisionConstants.PhotonConstants.BW_MAX_TAG_CLEAR_DIST = DefaultConstants.VisionConstants.PhotonConstants.BW_MAX_TAG_CLEAR_DIST;
        Constants.VisionConstants.PhotonConstants.OBJECT_GOAL_PITCH = DefaultConstants.VisionConstants.PhotonConstants.OBJECT_GOAL_PITCH;
        Constants.VisionConstants.PhotonConstants.OBJECT_DEGREE_TOLERANCE_TO_ACCEPT = DefaultConstants.VisionConstants.PhotonConstants.OBJECT_DEGREE_TOLERANCE_TO_ACCEPT;
        Constants.VisionConstants.QuestNavConstants.QUESTNAV_NT_NAME = DefaultConstants.VisionConstants.QuestNavConstants.QUESTNAV_NT_NAME;
        Constants.VisionConstants.QuestNavConstants.QUEST_FORWARD = DefaultConstants.VisionConstants.QuestNavConstants.QUEST_FORWARD;
        Constants.VisionConstants.QuestNavConstants.QUEST_LEFT = DefaultConstants.VisionConstants.QuestNavConstants.QUEST_LEFT;
        Constants.VisionConstants.QuestNavConstants.QUEST_UP = DefaultConstants.VisionConstants.QuestNavConstants.QUEST_UP;
        Constants.VisionConstants.QuestNavConstants.QUEST_ROLL = DefaultConstants.VisionConstants.QuestNavConstants.QUEST_ROLL;
        Constants.VisionConstants.QuestNavConstants.QUEST_PITCH = DefaultConstants.VisionConstants.QuestNavConstants.QUEST_PITCH;
        Constants.VisionConstants.QuestNavConstants.QUEST_YAW = DefaultConstants.VisionConstants.QuestNavConstants.QUEST_YAW;
        Constants.VisionConstants.QuestNavConstants.TRANSLATION_ERROR_TOLERANCE = DefaultConstants.VisionConstants.QuestNavConstants.TRANSLATION_ERROR_TOLERANCE;
        Constants.VisionConstants.QuestNavConstants.ROTATION_ERROR_TOLERANCE = DefaultConstants.VisionConstants.QuestNavConstants.ROTATION_ERROR_TOLERANCE;
        Constants.VisionConstants.QuestNavConstants.MIN_TAG_DIST_TO_BE_FAR = DefaultConstants.VisionConstants.QuestNavConstants.MIN_TAG_DIST_TO_BE_FAR;

        Constants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P = DefaultConstants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P;
        Constants.SwerveConstants.PATH_ROTATION_CONTROLLER_P = DefaultConstants.SwerveConstants.PATH_ROTATION_CONTROLLER_P;
        Constants.SwerveConstants.PATH_MANUAL_TRANSLATION_CONTROLLER = DefaultConstants.SwerveConstants.PATH_MANUAL_TRANSLATION_CONTROLLER;
        Constants.SwerveConstants.ANGULAR_POSITION_P = DefaultConstants.SwerveConstants.ANGULAR_POSITION_P;
        Constants.SwerveConstants.ANGULAR_POSITION_D = DefaultConstants.SwerveConstants.ANGULAR_POSITION_D;
        Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_P = DefaultConstants.SwerveConstants.ANGULAR_OBJECT_DETECTION_P;
        Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D = DefaultConstants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D;
        Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE = DefaultConstants.SwerveConstants.ANGULAR_MINIMUM_ANGLE;
        Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE = DefaultConstants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE;
        Constants.SwerveConstants.SWERVE_DRIVETRAIN_CONSTANTS = DefaultConstants.SwerveConstants.SWERVE_DRIVETRAIN_CONSTANTS;
        Constants.SwerveConstants.FRONT_LEFT = DefaultConstants.SwerveConstants.FrontLeft.FRONT_LEFT;
        Constants.SwerveConstants.FRONT_RIGHT = DefaultConstants.SwerveConstants.FrontRight.FRONT_RIGHT;
        Constants.SwerveConstants.BACK_LEFT = DefaultConstants.SwerveConstants.BackLeft.BACK_LEFT;
        Constants.SwerveConstants.BACK_RIGHT = DefaultConstants.SwerveConstants.BackRight.BACK_RIGHT;
    }

    private static void setSimConstants() {
        Constants.SwerveConstants.ANGULAR_POSITION_D = SimConstants.ANGULAR_POSITION_D;
        Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D = SimConstants.ANGULAR_OBJECT_DETECTION_D;
    }
}
