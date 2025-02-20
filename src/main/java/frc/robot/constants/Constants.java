package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Function;
import java.util.function.Supplier;

public final class Constants {

    // CAN bus that the devices are located on;
    // If there is more than one CAN bus, create a CANBus constant for each one
    public static CANBus CAN_BUS;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    public static Rotation2d BLUE_DEFAULT_ROTATION;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    public static Rotation2d RED_DEFAULT_ROTATION;

    public static Supplier<DriverStation.Alliance> ALLIANCE_SUPPLIER;

    // kSpeedAt12Volts desired top speed
    public static double MAX_VEL;
    // 1.96664381049 rotations per second tuned max angular velocity
    public static double MAX_ANGULAR_VEL;
    public static double MAX_CONTROLLED_ANGULAR_VEL;

    public static double MAX_ACCEL;
    public static double MAX_ANGULAR_ACCEL;
    public static double MAX_CONTROLLED_ACCEL;

    public static NetworkTableInstance NT_INSTANCE;
    public static int ONE_MILLION;
    public static double DEADBAND;

    public static final class AutoConstants {
        public static RobotConfig ROBOT_CONFIG;

        public static String ALGAE_CHECK_MARKER;

        public static PathConstraints PATH_CONSTRAINTS;

        public static double OBJECT_SEARCH_DEGREE_SLANT;
        public static double DEGREE_TOLERANCE_TO_ACCEPT;
        public static double TRANSLATION_TOLERANCE_TO_ACCEPT;
        public static double DISTANCE_TOLERANCE_TO_DRIVE_INTO;
    }

    public static final class VisionConstants {
        public static Matrix<N3, N1> ODOM_STD_DEV;
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION;
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_FUNCTION;

        public static final class LimelightConstants {
            public static String LIMELIGHT_NT_NAME;

            public static double LL_FORWARD;
            public static double LL_RIGHT;
            public static double LL_UP;
            public static double LL_ROLL;
            public static double LL_PITCH;
            public static double LL_YAW;

            public static double LL_MAX_TAG_CLEAR_DIST;
        }

        public static final class PhotonConstants {
            public static String BW_TOP_RIGHT_NAME;
            public static double BW_TOP_RIGHT_FORWARD;
            public static double BW_TOP_RIGHT_LEFT;
            public static double BW_TOP_RIGHT_UP;
            public static double BW_TOP_RIGHT_ROLL;
            public static double BW_TOP_RIGHT_PITCH;
            public static double BW_TOP_RIGHT_YAW;

            public static String BW_TOP_LEFT_NAME;
            public static double BW_TOP_LEFT_FORWARD;
            public static double BW_TOP_LEFT_LEFT;
            public static double BW_TOP_LEFT_UP;
            public static double BW_TOP_LEFT_ROLL;
            public static double BW_TOP_LEFT_PITCH;
            public static double BW_TOP_LEFT_YAW;

            public static String BW_BACK_NAME;
            public static double BW_BACK_FORWARD;
            public static double BW_BACK_LEFT;
            public static double BW_BACK_UP;
            public static double BW_BACK_ROLL;
            public static double BW_BACK_PITCH;
            public static double BW_BACK_YAW;

            public static double BW_MAX_TAG_CLEAR_DIST;

            public static double OBJECT_GOAL_PITCH;
        }

        public static final class QuestNavConstants {
            public static String QUESTNAV_NT_NAME;

            public static double QUEST_FORWARD;
            public static double QUEST_LEFT;
            public static double QUEST_UP;
            public static double QUEST_ROLL;
            public static double QUEST_PITCH;
            public static double QUEST_YAW;

            // The thresholds through which the QuestNav's correctional offset will be recorrected by the error amount.
            public static double TRANSLATION_ERROR_TOLERANCE;
            public static double ROTATION_ERROR_TOLERANCE;

            public static double MIN_TAG_DIST_TO_BE_FAR;
        }
    }

    public final static class ElevatorConstants {
        // basic configs
        public static int LEAD_ID;
        public static int FOLLOWER_ID;
        public static int LOWER_LIMIT_SWITCH_ID;
        public static int CURRENT_LIMIT;
        public static InvertedValue ELEVATOR_INVERT;

        // pid
        public static double ELEVATOR_S;
        public static double ELEVATOR_V;
        public static double ELEVATOR_A;
        public static double ELEVATOR_P;
        public static double ELEVATOR_I;
        public static double ELEVATOR_D;

        // presets
        public static double LOWER_LIMIT;
        public static double UPPER_LIMIT;
        public static double GROUND_CORAL;
        public static double GROUND_ALGAE;
        public static double L1_CORAL;
        public static double L2_CORAL;
        public static double L3_CORAL;
        public static double L4_CORAL;
        public static double LOW_REEF_ALGAE;
        public static double HIGH_REEF_ALGAE;
        public static double PROCESSOR;
        public static double NET;

    }

    public final static class IntakeConstants {
        // basic configs
        public static int MOTOR_ID;
        public static int CORAL_BEAM_ID;
        public static int ALGAE_BEAM_ID;
        public static int CURRENT_LIMIT;
        public static InvertedValue INVERT;
    }

    public final static class PivotConstants {
        // basic configs
        public static int LEAD_ID;
        public static int FOLLOWER_ID;
        public static int ENCODER_ID;
        public static int LOWER_LIMIT_SWITCH_ID;
        public static int UPPER_LIMIT_SWITCH_ID;
        public static int RATCHET_ID;
        public static int CURRENT_LIMIT;
        public static InvertedValue PIVOT_INVERT;

        // pid
        public static double PIVOT_S;
        public static double PIVOT_V;
        public static double PIVOT_A;
        public static double PIVOT_P;
        public static double PIVOT_I;
        public static double PIVOT_D;

        // presets
        public static double LOWER_LIMIT;
        public static double UPPER_LIMIT;
        public static double CORAL_STATION;
        public static double GROUND_ALGAE;
        public static double GROUND_CORAL;
        public static double SCORE_CORAL;
        public static double SCORE_ALGAE;
        public static double STOW_POSITION;
        public static double TOLERANCE;

        public static double RATCHET_ON;
        public static double RATCHET_OFF;
    }

    public final static class WristConstants {
        // basic configs
        public static int MOTOR_ID;
        public static int ENCODER_ID;
        public static int LOWER_LIMIT_SWITCH_ID;
        public static int UPPER_LIMIT_SWITCH_ID;
        public static int CURRENT_LIMIT;
        public static InvertedValue WRIST_INVERT;

        // pid
        public static double WRIST_S;
        public static double WRIST_V;
        public static double WRIST_A;
        public static double WRIST_P;
        public static double WRIST_I;
        public static double WRIST_D;

        // presets
        public static double LOWER_LIMIT;
        public static double UPPER_LIMIT; 
        public static double GROUND_CORAL;
        public static double GROUND_ALGAE;
        public static double L1_CORAL;
        public static double L2_L3_CORAL;
        public static double L4_CORAL;
        public static double REEF_ALGAE;
        public static double PROCESSOR;
        public static double NET;

    }

    public static final class SwerveConstants {
        public static double PATH_TRANSLATION_CONTROLLER_P;
        public static double PATH_ROTATION_CONTROLLER_P;

        public static double TRANSLATION_ALIGNMENT_CONTROLLER_P;
        public static double TRANSLATION_ALIGNMENT_CONTROLLER_D;

        public static Function<Double, Double> PATH_MANUAL_TRANSLATION_CONTROLLER;

        public static double ANGULAR_POSITION_P;
        public static double ANGULAR_POSITION_D;

        public static double ANGULAR_OBJECT_DETECTION_P;
        public static double ANGULAR_OBJECT_DETECTION_D;

        public static double ANGULAR_MINIMUM_ANGLE;
        public static double ANGULAR_MAXIMUM_ANGLE;

        public static AudioConfigs AUDIO_CONFIGS;

        public static SwerveDrivetrainConstants SWERVE_DRIVETRAIN_CONSTANTS;

        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT;
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT;
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT;
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT;
    }
}
