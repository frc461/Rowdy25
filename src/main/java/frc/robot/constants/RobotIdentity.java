package frc.robot.constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.constants.variants.DefaultConstants;
import frc.robot.constants.variants.SimConstants;
import frc.robot.constants.variants.TestConstants;

public enum RobotIdentity {
    ROWDY1,
    ROWDY2,
    TEST,
    SIM;

    private static RobotIdentity getIdentity() {
        String mac = MacAddress.getMACAddress();
        if (mac.equals(MacAddress.ROWDY1)) {
            return ROWDY1;
        } else if (mac.equals(MacAddress.ROWDY2)) {
            return ROWDY2;
        } else if (mac.equals(MacAddress.TEST)) {
            return TEST;
        }
        return SIM;
    }

    public static void initializeConstants() {
        setDefaultConstants();
        NetworkTable identityEntry = Constants.NT_INSTANCE.getTable("Robot");
        StringPublisher identityPublisher = identityEntry.getStringTopic("Robot Identity").publish();
        switch (getIdentity()) {
            case ROWDY1:
                identityPublisher.set(ROWDY1.name());
                break;
            case ROWDY2:
                identityPublisher.set(ROWDY2.name());
                break;
            case TEST:
                setTestConstants();
                identityPublisher.set(TEST.name());
                break;
            case SIM:
                setSimConstants();
                identityPublisher.set(SIM.name());
                break;
        }
    }

    private static void setDefaultConstants() {
        Constants.CAN_BUS = DefaultConstants.CAN_BUS;
        Constants.BLUE_DEFAULT_ROTATION = DefaultConstants.BLUE_DEFAULT_ROTATION;
        Constants.RED_DEFAULT_ROTATION = DefaultConstants.RED_DEFAULT_ROTATION;
        Constants.ALLIANCE_SUPPLIER = DefaultConstants.ALLIANCE_SUPPLIER;
        Constants.MAX_VEL = DefaultConstants.MAX_VEL;
        Constants.MAX_ANGULAR_VEL = DefaultConstants.MAX_ANGULAR_VEL;
        Constants.MAX_CONTROLLED_ANGULAR_VEL = DefaultConstants.MAX_CONTROLLED_ANGULAR_VEL;
        Constants.MAX_ACCEL = DefaultConstants.MAX_ACCEL;
        Constants.MAX_ANGULAR_ACCEL = DefaultConstants.MAX_ANGULAR_ACCEL;
        Constants.MAX_CONTROLLED_ACCEL = DefaultConstants.MAX_CONTROLLED_ACCEL;
        Constants.NT_INSTANCE = DefaultConstants.NT_INSTANCE;
        Constants.ONE_MILLION = DefaultConstants.ONE_MILLION;
        Constants.DEADBAND = DefaultConstants.DEADBAND;

        Constants.AutoConstants.ROBOT_CONFIG = DefaultConstants.AutoConstants.ROBOT_CONFIG;
        Constants.AutoConstants.ALGAE_CHECK_MARKER = DefaultConstants.AutoConstants.ALGAE_CHECK_MARKER;
        Constants.AutoConstants.PATH_CONSTRAINTS = DefaultConstants.AutoConstants.PATH_CONSTRAINTS;
        Constants.AutoConstants.OBJECT_SEARCH_DEGREE_SLANT = DefaultConstants.AutoConstants.OBJECT_SEARCH_DEGREE_SLANT;
        Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT = DefaultConstants.AutoConstants.DEGREE_TOLERANCE_TO_DRIVE_INTO;
        Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT = DefaultConstants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        Constants.AutoConstants.DISTANCE_TOLERANCE_TO_DRIVE_INTO = DefaultConstants.AutoConstants.DISTANCE_TOLERANCE_TO_DRIVE_INTO;

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
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_NAME = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_NAME;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_FORWARD = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_FORWARD;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_LEFT = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_LEFT;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_UP = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_UP;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_ROLL = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_ROLL;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_PITCH = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_PITCH;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_YAW = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_YAW;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_NAME = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_NAME;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_FORWARD = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_FORWARD;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_LEFT = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_LEFT;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_UP = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_UP;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_ROLL = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_ROLL;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_PITCH = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_PITCH;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_YAW = DefaultConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_YAW;
        Constants.VisionConstants.PhotonConstants.BW_BACK_NAME = DefaultConstants.VisionConstants.PhotonConstants.BW_BACK_NAME;
        Constants.VisionConstants.PhotonConstants.BW_BACK_FORWARD = DefaultConstants.VisionConstants.PhotonConstants.BW_BACK_FORWARD;
        Constants.VisionConstants.PhotonConstants.BW_BACK_LEFT = DefaultConstants.VisionConstants.PhotonConstants.BW_BACK_LEFT;
        Constants.VisionConstants.PhotonConstants.BW_BACK_UP = DefaultConstants.VisionConstants.PhotonConstants.BW_BACK_UP;
        Constants.VisionConstants.PhotonConstants.BW_BACK_ROLL = DefaultConstants.VisionConstants.PhotonConstants.BW_BACK_ROLL;
        Constants.VisionConstants.PhotonConstants.BW_BACK_PITCH = DefaultConstants.VisionConstants.PhotonConstants.BW_BACK_PITCH;
        Constants.VisionConstants.PhotonConstants.BW_BACK_YAW = DefaultConstants.VisionConstants.PhotonConstants.BW_BACK_YAW;
        Constants.VisionConstants.PhotonConstants.BW_MAX_TAG_CLEAR_DIST = DefaultConstants.VisionConstants.PhotonConstants.BW_MAX_TAG_CLEAR_DIST;
        Constants.VisionConstants.PhotonConstants.OBJECT_GOAL_PITCH = DefaultConstants.VisionConstants.PhotonConstants.OBJECT_GOAL_PITCH;
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

        Constants.ElevatorConstants.LEAD_ID = DefaultConstants.ElevatorConstants.LEAD_ID;
        Constants.ElevatorConstants.FOLLOWER_ID = DefaultConstants.ElevatorConstants.FOLLOWER_ID;
        Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_ID = DefaultConstants.ElevatorConstants.LOWER_LIMIT_SWITCH_ID;
        Constants.ElevatorConstants.CURRENT_LIMIT = DefaultConstants.ElevatorConstants.CURRENT_LIMIT;
        Constants.ElevatorConstants.ELEVATOR_INVERT = DefaultConstants.ElevatorConstants.ELEVATOR_INVERT;
        Constants.ElevatorConstants.ELEVATOR_S = DefaultConstants.ElevatorConstants.ELEVATOR_S;
        Constants.ElevatorConstants.ELEVATOR_V = DefaultConstants.ElevatorConstants.ELEVATOR_V;
        Constants.ElevatorConstants.ELEVATOR_A = DefaultConstants.ElevatorConstants.ELEVATOR_A;
        Constants.ElevatorConstants.ELEVATOR_P = DefaultConstants.ElevatorConstants.ELEVATOR_P;
        Constants.ElevatorConstants.ELEVATOR_I = DefaultConstants.ElevatorConstants.ELEVATOR_I;
        Constants.ElevatorConstants.ELEVATOR_D = DefaultConstants.ElevatorConstants.ELEVATOR_D;
        Constants.ElevatorConstants.LOWER_LIMIT = DefaultConstants.ElevatorConstants.LOWER_LIMIT;
        Constants.ElevatorConstants.UPPER_LIMIT = DefaultConstants.ElevatorConstants.UPPER_LIMIT;

        Constants.IntakeConstants.MOTOR_ID = DefaultConstants.IntakeConstants.MOTOR_ID;
        Constants.IntakeConstants.SENSOR_ID = DefaultConstants.IntakeConstants.SENSOR_ID;
        Constants.IntakeConstants.CURRENT_LIMIT = DefaultConstants.IntakeConstants.CURRENT_LIMIT;
        Constants.IntakeConstants.PEAK_VOLTAGE = DefaultConstants.IntakeConstants.PEAK_VOLTAGE;
        Constants.IntakeConstants.MOTOR_INVERT = DefaultConstants.IntakeConstants.MOTOR_INVERT;
        Constants.IntakeConstants.NEUTRAL_MODE = DefaultConstants.IntakeConstants.NEUTRAL_MODE;

        Constants.PivotConstants.LEAD_ID = DefaultConstants.PivotConstants.LEAD_ID;
        Constants.PivotConstants.FOLLOWER_ID = DefaultConstants.PivotConstants.FOLLOWER_ID;
        Constants.PivotConstants.ENCODER_ID = DefaultConstants.PivotConstants.ENCODER_ID;
        Constants.PivotConstants.LOWER_LIMIT_SWITCH_ID = DefaultConstants.PivotConstants.LOWER_LIMIT_SWITCH_ID;
        Constants.PivotConstants.UPPER_LIMIT_SWITCH_ID = DefaultConstants.PivotConstants.UPPER_LIMIT_SWITCH_ID;
        Constants.PivotConstants.RATCHET_ID = DefaultConstants.PivotConstants.RATCHET_ID;
        Constants.PivotConstants.CURRENT_LIMIT = DefaultConstants.PivotConstants.CURRENT_LIMIT;
        Constants.PivotConstants.PIVOT_INVERT = DefaultConstants.PivotConstants.PIVOT_INVERT;
        Constants.PivotConstants.PIVOT_S = DefaultConstants.PivotConstants.PIVOT_S;
        Constants.PivotConstants.PIVOT_V = DefaultConstants.PivotConstants.PIVOT_V;
        Constants.PivotConstants.PIVOT_A = DefaultConstants.PivotConstants.PIVOT_A;
        Constants.PivotConstants.PIVOT_P = DefaultConstants.PivotConstants.PIVOT_P;
        Constants.PivotConstants.PIVOT_I = DefaultConstants.PivotConstants.PIVOT_I;
        Constants.PivotConstants.PIVOT_D = DefaultConstants.PivotConstants.PIVOT_D;
        Constants.PivotConstants.LOWER_LIMIT = DefaultConstants.PivotConstants.LOWER_LIMIT;
        Constants.PivotConstants.UPPER_LIMIT = DefaultConstants.PivotConstants.UPPER_LIMIT;
        Constants.PivotConstants.STOW_POSITION = DefaultConstants.PivotConstants.STOW_POSITION;
        Constants.PivotConstants.TOLERANCE = DefaultConstants.PivotConstants.TOLERANCE;
        Constants.PivotConstants.RATCHET_ON = DefaultConstants.PivotConstants.RATCHET_ON;
        Constants.PivotConstants.RATCHET_OFF = DefaultConstants.PivotConstants.RATCHET_OFF;

        Constants.WristConstants.MOTOR_ID = DefaultConstants.WristConstants.MOTOR_ID;
        Constants.WristConstants.CURRENT_LIMIT = DefaultConstants.WristConstants.CURRENT_LIMIT;
        Constants.WristConstants.PEAK_VOLTAGE = DefaultConstants.WristConstants.PEAK_VOLTAGE;
        Constants.WristConstants.SENSOR_TO_DEGREE_RATIO = DefaultConstants.WristConstants.SENSOR_TO_DEGREE_RATIO;
        Constants.WristConstants.MOTOR_INVERT = DefaultConstants.WristConstants.MOTOR_INVERT;
        Constants.WristConstants.NEUTRAL_MODE = DefaultConstants.WristConstants.NEUTRAL_MODE;
        Constants.WristConstants.ENCODER_ID = DefaultConstants.WristConstants.ENCODER_ID;
        Constants.WristConstants.ENCODER_ABSOLUTE_OFFSET = DefaultConstants.WristConstants.ENCODER_ABSOLUTE_OFFSET;
        Constants.WristConstants.ENCODER_INVERT = DefaultConstants.WristConstants.ENCODER_INVERT;
        Constants.WristConstants.G = DefaultConstants.WristConstants.G;
        Constants.WristConstants.V = DefaultConstants.WristConstants.V;
        Constants.WristConstants.A = DefaultConstants.WristConstants.A;
        Constants.WristConstants.P = DefaultConstants.WristConstants.P;
        Constants.WristConstants.I = DefaultConstants.WristConstants.I;
        Constants.WristConstants.D = DefaultConstants.WristConstants.D;
        Constants.WristConstants.EXPO_V = DefaultConstants.WristConstants.EXPO_V;
        Constants.WristConstants.EXPO_A = DefaultConstants.WristConstants.EXPO_A;
        Constants.WristConstants.LOWER_LIMIT = DefaultConstants.WristConstants.LOWER_LIMIT;
        Constants.WristConstants.UPPER_LIMIT = DefaultConstants.WristConstants.UPPER_LIMIT;
        Constants.WristConstants.GROUND_CORAL = DefaultConstants.WristConstants.GROUND_CORAL;
        Constants.WristConstants.GROUND_ALGAE = DefaultConstants.WristConstants.GROUND_ALGAE;
        Constants.WristConstants.L1_CORAL = DefaultConstants.WristConstants.L1_CORAL;
        Constants.WristConstants.L2_L3_CORAL = DefaultConstants.WristConstants.L2_L3_CORAL;
        Constants.WristConstants.L4_CORAL = DefaultConstants.WristConstants.L4_CORAL;
        Constants.WristConstants.REEF_ALGAE = DefaultConstants.WristConstants.REEF_ALGAE;
        Constants.WristConstants.PROCESSOR = DefaultConstants.WristConstants.PROCESSOR;
        Constants.WristConstants.NET = DefaultConstants.WristConstants.NET;
        Constants.WristConstants.STOW_POSITION = DefaultConstants.WristConstants.STOW_POSITION;

        Constants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P = DefaultConstants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P;
        Constants.SwerveConstants.PATH_ROTATION_CONTROLLER_P = DefaultConstants.SwerveConstants.PATH_ROTATION_CONTROLLER_P;
        Constants.SwerveConstants.TRANSLATION_ALIGNMENT_CONTROLLER_P = DefaultConstants.SwerveConstants.TRANSLATION_ALIGNMENT_CONTROLLER_P;
        Constants.SwerveConstants.TRANSLATION_ALIGNMENT_CONTROLLER_D = DefaultConstants.SwerveConstants.TRANSLATION_ALIGNMENT_CONTROLLER_D;
        Constants.SwerveConstants.PATH_MANUAL_TRANSLATION_CONTROLLER = DefaultConstants.SwerveConstants.PATH_MANUAL_TRANSLATION_CONTROLLER;
        Constants.SwerveConstants.ANGULAR_POSITION_P = DefaultConstants.SwerveConstants.ANGULAR_POSITION_P;
        Constants.SwerveConstants.ANGULAR_POSITION_D = DefaultConstants.SwerveConstants.ANGULAR_POSITION_D;
        Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_P = DefaultConstants.SwerveConstants.ANGULAR_OBJECT_DETECTION_P;
        Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D = DefaultConstants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D;
        Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE = DefaultConstants.SwerveConstants.ANGULAR_MINIMUM_ANGLE;
        Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE = DefaultConstants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE;
        Constants.SwerveConstants.AUDIO_CONFIGS = DefaultConstants.SwerveConstants.AUDIO_CONFIGS;
        Constants.SwerveConstants.SWERVE_DRIVETRAIN_CONSTANTS = DefaultConstants.SwerveConstants.SWERVE_DRIVETRAIN_CONSTANTS;
        Constants.SwerveConstants.FRONT_LEFT = DefaultConstants.SwerveConstants.FrontLeft.FRONT_LEFT;
        Constants.SwerveConstants.FRONT_RIGHT = DefaultConstants.SwerveConstants.FrontRight.FRONT_RIGHT;
        Constants.SwerveConstants.BACK_LEFT = DefaultConstants.SwerveConstants.BackLeft.BACK_LEFT;
        Constants.SwerveConstants.BACK_RIGHT = DefaultConstants.SwerveConstants.BackRight.BACK_RIGHT;
    }

    private static void setTestConstants() {
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_NAME = TestConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_NAME;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_FORWARD = TestConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_FORWARD;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_LEFT = TestConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_LEFT;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_UP = TestConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_UP;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_ROLL = TestConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_ROLL;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_PITCH = TestConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_PITCH;
        Constants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_YAW = TestConstants.VisionConstants.PhotonConstants.BW_TOP_RIGHT_YAW;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_NAME = TestConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_NAME;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_FORWARD = TestConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_FORWARD;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_LEFT = TestConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_LEFT;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_UP = TestConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_UP;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_ROLL = TestConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_ROLL;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_PITCH = TestConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_PITCH;
        Constants.VisionConstants.PhotonConstants.BW_TOP_LEFT_YAW = TestConstants.VisionConstants.PhotonConstants.BW_TOP_LEFT_YAW;

        Constants.SwerveConstants.SWERVE_DRIVETRAIN_CONSTANTS = TestConstants.SwerveConstants.SWERVE_DRIVETRAIN_CONSTANTS;
        Constants.SwerveConstants.FRONT_LEFT = TestConstants.SwerveConstants.FrontLeft.FRONT_LEFT;
        Constants.SwerveConstants.FRONT_RIGHT = TestConstants.SwerveConstants.FrontRight.FRONT_RIGHT;
        Constants.SwerveConstants.BACK_LEFT = TestConstants.SwerveConstants.BackLeft.BACK_LEFT;
        Constants.SwerveConstants.BACK_RIGHT = TestConstants.SwerveConstants.BackRight.BACK_RIGHT;
    }

    private static void setSimConstants() {
        Constants.SwerveConstants.ANGULAR_POSITION_D = SimConstants.ANGULAR_POSITION_D;
        Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D = SimConstants.ANGULAR_OBJECT_DETECTION_D;
    }
}
