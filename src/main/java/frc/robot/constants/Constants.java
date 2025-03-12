package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.BiFunction;
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

    public static Distance ROBOT_LENGTH_WITH_BUMPERS;
    public static Distance ROBOT_WIDTH_WITH_BUMPERS;

    public static Supplier<DriverStation.Alliance> ALLIANCE_SUPPLIER;

    public static Function<Supplier<DriverStation.Alliance>, Pose2d> CENTER_OF_LEFT_CORAL_STATION;
    public static Function<Supplier<DriverStation.Alliance>, Pose2d> CENTER_OF_RIGHT_CORAL_STATION;
    public static Function<Supplier<DriverStation.Alliance>, Pose2d> FAR_LEFT_CORAL_STATION;
    public static Function<Supplier<DriverStation.Alliance>, Pose2d> FAR_RIGHT_CORAL_STATION;

    // kSpeedAt12Volts desired top speed
    public static double MAX_VEL;
    public static Function<Double, Double> MAX_CONTROLLED_VEL;
    // 1.96664381049 rotations per second tuned max angular velocity
    public static Function<Double, Double> MAX_ANGULAR_VEL;
    public static Function<Double, Double> MAX_CONTROLLED_ANGULAR_VEL;

    public static double MAX_ACCEL;
    public static double MAX_ANGULAR_ACCEL;
    public static double MAX_CONTROLLED_ACCEL;

    public static NetworkTableInstance NT_INSTANCE;
    public static int ONE_MILLION;
    public static double DEADBAND;

    public static int SERVO_HUB_ID;
    public static ServoHub SERVO_HUB;

    public static final class AutoConstants {
        public static RobotConfig ROBOT_CONFIG;

        public static String ALGAE_CHECK_MARKER;
        public static String INTAKE_MARKER;
        public static String OUTTAKE_MARKER;

        public static PathConstraints PATH_CONSTRAINTS;

        public static double OBJECT_SEARCH_DEGREE_SLANT;
        public static double DEGREE_TOLERANCE_TO_ACCEPT;
        public static double TRANSLATION_TOLERANCE_TO_ACCEPT;
        public static double TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE;
        public static double TRANSLATION_TOLERANCE_TO_TRANSITION;
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

            public static double OBJECT_TARGET_PITCH;
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

    public final static class ClimbConstants {
        // motor config
        public static int ID;
        public static double CURRENT_LIMIT;
        public static InvertedValue MOTOR_INVERT;
        public static NeutralModeValue NEUTRAL_MODE;

        // ratchet
        public static ServoChannel.ChannelId RATCHET_CHANNEL;
        public static int RATCHET_ON;
        public static int RATCHET_OFF;

        // presets
        public static double STOW;
        public static double PREPARE_CLIMB;
        public static double CLIMB;
    }

    public final static class ElevatorConstants {
        // motor config
        public static int LEAD_ID;
        public static int FOLLOWER_ID;
        public static int LOWER_LIMIT_SWITCH_ID;
        public static double CURRENT_LIMIT;
        public static InvertedValue MOTOR_INVERT;
        public static NeutralModeValue NEUTRAL_MODE;

        // mechanism characterization
        public static double ROTOR_TO_INCH_RATIO;
        public static double MASS_LBS;
        public static double COM_TO_STAGE_2_RATIO;
        public static double STAGE_3_LIMIT;
        public static double COM_TO_STAGE_3_RATIO;
        public static Translation2d ZERO_UPRIGHT_COM;

        // pid & tolerance
        public static Function<Double, Double> G;
        public static double V;
        public static double A;
        public static double P;
        public static double I;
        public static double D;
        public static double EXPO_V;
        public static double EXPO_A;
        public static double SAFE_TOLERANCE;
        public static double AT_TARGET_TOLERANCE;

        // presets
        public static double LOWER_LIMIT;
        public static double UPPER_LIMIT;
        public static double STOW;
        public static double CORAL_STATION;
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
        public static double CLIMB;
    }

    public final static class IntakeConstants {
        // basic configs
        public static int MOTOR_ID;
        public static int SENSOR_ID;
        public static int BEAMBREAK_ID;
        public static double CURRENT_LIMIT;
        public static InvertedValue MOTOR_INVERT;
        public static NeutralModeValue NEUTRAL_MODE;
        public static double DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD;
    }

    public final static class PivotConstants {
        // motor config
        public static int LEAD_ID;
        public static int FOLLOWER_ID;
        public static double CURRENT_LIMIT;
        public static InvertedValue PIVOT_INVERT;
        public static NeutralModeValue NEUTRAL_MODE;

        // mechanism characterization
        public static double SENSOR_TO_DEGREE_RATIO;
        public static Translation2d AXIS_POSITION;

        // encoder config
        public static int ENCODER_ID;
        public static double ENCODER_ABSOLUTE_OFFSET;
        public static SensorDirectionValue ENCODER_INVERT;

        // ratchet config
        public static ServoChannel.ChannelId RATCHET_CHANNEL;
        public static int RATCHET_ON;
        public static int RATCHET_OFF;

        // pid & tolerance
        public static double G;
        public static double V;
        public static double A;
        public static double P;
        public static double I;
        public static double D;
        public static double EXPO_V;
        public static double EXPO_A;
        public static double SAFE_TOLERANCE;
        public static double AT_TARGET_TOLERANCE;

        // presets
        public static double LOWER_LIMIT;
        public static double UPPER_LIMIT;
        public static double STOW;
        public static double CORAL_STATION;
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
        public static double CLIMB;
    }

    public final static class WristConstants {
        // motor config
        public static int MOTOR_ID;
        public static double CURRENT_LIMIT;
        public static InvertedValue MOTOR_INVERT;
        public static NeutralModeValue NEUTRAL_MODE;

        // mechanism characterization
        public static double SENSOR_TO_DEGREE_RATIO;
        public static double MASS_LBS;
        public static Translation2d AXIS_POSITION;
        public static Translation2d AXIS_TO_ZERO_COM;

        // encoder config
        public static int ENCODER_ID;
        public static double ENCODER_ABSOLUTE_OFFSET;
        public static SensorDirectionValue ENCODER_INVERT;

        // pid & tolerance
        public static BiFunction<Double, Double, Double> G;
        public static double V;
        public static double A;
        public static double P;
        public static double I;
        public static double D;
        public static double EXPO_V;
        public static double EXPO_A;
        public static double SAFE_TOLERANCE;
        public static double AT_TARGET_TOLERANCE;

        // presets
        public static BiFunction<Double, Double, Double> LOWER_LIMIT;
        public static Function<Double, Double> UPPER_LIMIT;
        public static double STOW;
        public static double CORAL_STATION;
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
        public static double CLIMB;
    }

    public static final class SwerveConstants {
        public static double PATH_TRANSLATION_CONTROLLER_P; // FOR PATHPLANNER
        public static double PATH_ROTATION_CONTROLLER_P;

        public static double ANGULAR_POSITION_P; // FOR ANGULAR ROTATION CONTROL
        public static double ANGULAR_POSITION_D;

        public static double ANGULAR_OBJECT_DETECTION_P; // FOR OBJECT DETECTION ROTATION CONTROL
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
