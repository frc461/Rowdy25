package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
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
    public static final String Elevator = null;

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
    public static double MAX_REAL_ANGULAR_VEL;
    public static double MAX_CONTROLLED_ANGULAR_VEL;

    public static double MAX_ACCEL;
    public static double MAX_CONTROLLED_ACCEL;
    public static double MAX_ANGULAR_ACCEL;

    public static NetworkTableInstance NT_INSTANCE;
    public static int ONE_MILLION;
    public static double STICK_DEADBAND;

    public static final class AutoConstants {
        public static RobotConfig ROBOT_CONFIG;

        // TODO: REMOVE ALL MENTIONS OF "NOTE"
        public static String NOTE_CHECK_MARKER;

        public static PathConstraints PATH_CONSTRAINTS;

        public static double NOTE_SEARCH_DEGREE_SLANT;
        public static double DEGREE_TOLERANCE_TO_ACCEPT;
        public static double TRANSLATION_TOLERANCE_TO_ACCEPT;
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
            public static double BW_FORWARD;
            public static double BW_LEFT;
            public static double BW_UP;
            public static double BW_ROLL;
            public static double BW_PITCH;
            public static double BW_YAW;

            public static double BW_MAX_TAG_CLEAR_DIST;

            public static double OBJECT_GOAL_PITCH;
            public static double OBJECT_DEGREE_TOLERANCE_TO_ACCEPT;
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

    // TODO: MOVE TO DEFAULT CONSTANTS VARIANT + UPDATE VALUES FOR 2025 + TUNE
    public final static class ElevatorConstants {
        // basic configs
        public static final int LEAD_ID = 31;
        public static final int FOLLOWER_ID = 32;
        public static final int CURRENT_LIMIT = 80;
        public static final int SERVO_LIMIT_SWITCH = 1;
        public static final int LOWER_LIMIT_SWITCH_ID = 2;
        public static final InvertedValue ELEVATOR_INVERT = InvertedValue.Clockwise_Positive;

        // servo to hold elevator in endgame
        public static final int ELEVATOR_SERVO_PORT = 1;
        public static final double ELEVATOR_SERVO_CLAMPED_POS = .07; // new servo values
        public static final double ELEVATOR_SERVO_UNCLAMPED_POS = .39;

        // pid
        public static final double ELEVATOR_P = 0.05;
        public static final double ELEVATOR_I = 0.0;
        public static final double ELEVATOR_D = 0.0;

        // presets
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 37;
    }

    public final static class WristConstants {
        // basic configs
        public static final int MOTOR_ID = 62;
        public static final int CURRENT_LIMIT = 35;
        public static final InvertedValue WRIST_INVERT = InvertedValue.Clockwise_Positive;

        // pid for angler
        public static final double WRIST_P = 0.2;
        public static final double WRIST_I = 0.00001;
        public static final double WRIST_D = 0.0001;

        // limit switches
        public static final int LOWER_LIMIT_SWITCH_ID = 6;

        // code limits on encoder values
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 20;
        public static final int UPPER_LIMIT_SWITCH_ID = 0;
    }

    public final static class IntakeConstants {
        public static final int LEFT_MOTOR_ID = 11;
        public static final int RIGHT_MOTOR_ID = 12;
        public static final InvertedValue LEFT_INVERT = InvertedValue.Clockwise_Positive; //TODO: CHECK ON REAL ROBOT
        public static final InvertedValue RIGHT_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final int CURRENT_LIMIT = 40;
        public static final int CORAL_BEAM_ID = 3;
        public static final int ALGAE_BEAM_ID = 4;
    }

    public final static class PivotConstants {

        public static final int MOTOR_ID = 0;
        public static final InvertedValue PIVOT_INVERT = InvertedValue.Clockwise_Positive;
        public static final int CURRENT_LIMIT = 0;
        public static final int LOWER_LIMIT_SWITCH_ID = 0;
        
        
        public static final double PIVOT_P = 0;
        public static final double PIVOT_I = 0;
        public static final double PIVOT_D = 0;
  

        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 0;
        public static final int UPPER_LIMIT_SWITCH_ID = 0;
    
    }

    public static final class SwerveConstants {
        public static double PATH_TRANSLATION_CONTROLLER_P;
        public static double PATH_ROTATION_CONTROLLER_P;

        public static Function<Double, Double> PATH_MANUAL_TRANSLATION_CONTROLLER;

        public static double ANGULAR_POSITION_P;
        public static double ANGULAR_POSITION_D;

        public static double ANGULAR_OBJECT_DETECTION_P;
        public static double ANGULAR_OBJECT_DETECTION_D;

        public static double ANGULAR_MINIMUM_ANGLE;
        public static double ANGULAR_MAXIMUM_ANGLE;

        public static SwerveDrivetrainConstants SWERVE_DRIVETRAIN_CONSTANTS;

        public static SwerveModuleConstants FRONT_LEFT;
        public static SwerveModuleConstants FRONT_RIGHT;
        public static SwerveModuleConstants BACK_LEFT;
        public static SwerveModuleConstants BACK_RIGHT;
    }
}
