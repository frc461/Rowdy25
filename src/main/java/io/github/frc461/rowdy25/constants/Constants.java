package io.github.frc461.rowdy25.constants;

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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Central repository for all robot-wide constants.
 * <p>
 * All fields are assigned at runtime by {@link RobotIdentity#initializeConstants()},
 * which detects the robot hardware identity and loads the appropriate values from
 * {@link io.github.frc461.rowdy25.constants.variants.DefaultConstants} with overrides
 * from robot-specific variant classes. This indirection allows the same compiled code
 * to run on multiple physical robot instances with different hardware configurations.
 *
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Aneesh Terani, <a href="https://github.com/aterani">GitHub</a>
 * @author JiuJiu Liu, <a href="https://github.com/jooj99">GitHub</a>
 * @author Leo Minton, <a href="https://github.com/leo-minton">GitHub</a>
 */
public final class Constants {

    /** The detected robot identity, populated during initialization. */
    public static RobotIdentity IDENTITY;

    // CAN bus that the devices are located on;
    // If there is more than one CAN bus, create a CANBus constant for each one
    /** The CAN bus used by the drivetrain and subsystem devices. */
    public static CANBus CAN_BUS;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    /** Default rotation for the blue alliance (facing the field). */
    public static Rotation2d BLUE_DEFAULT_ROTATION;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    /** Default rotation for the red alliance (facing the field). */
    public static Rotation2d RED_DEFAULT_ROTATION;

    /** Length of the robot chassis including bumpers. */
    public static Distance ROBOT_LENGTH_WITH_BUMPERS;
    /** Width of the robot chassis including bumpers. */
    public static Distance ROBOT_WIDTH_WITH_BUMPERS;

    /** Supplier for the current DriverStation alliance. */
    public static Supplier<DriverStation.Alliance> ALLIANCE_SUPPLIER;

    /** Computes the center-left coral station scoring pose based on alliance. */
    public static Function<Supplier<DriverStation.Alliance>, Pose2d> CENTER_OF_LEFT_CORAL_STATION;
    /** Computes the center-right coral station scoring pose based on alliance. */
    public static Function<Supplier<DriverStation.Alliance>, Pose2d> CENTER_OF_RIGHT_CORAL_STATION;
    /** Computes the far-left coral station scoring pose based on alliance. */
    public static Function<Supplier<DriverStation.Alliance>, Pose2d> FAR_LEFT_CORAL_STATION;
    /** Computes the far-right coral station scoring pose based on alliance. */
    public static Function<Supplier<DriverStation.Alliance>, Pose2d> FAR_RIGHT_CORAL_STATION;

    // kSpeedAt12Volts desired top speed
    /** Maximum translational velocity at 12V (m/s). */
    public static double MAX_VEL;
    /** Maximum controlled velocity as a function of elevator height (m/s). */
    public static Function<Double, Double> MAX_CONTROLLED_VEL;
    // 1.96664381049 rotations per second tuned max angular velocity
    /** Maximum angular velocity as a function of elevator height (rad/s). */
    public static Function<Double, Double> MAX_ANGULAR_VEL;
    /** Maximum controlled angular velocity as a function of elevator height (rad/s). */
    public static Function<Double, Double> MAX_CONTROLLED_ANGULAR_VEL;

    /** Maximum translational acceleration (m/s^2). */
    public static double MAX_ACCEL;
    /** Maximum angular acceleration (rad/s^2). */
    public static double MAX_ANGULAR_ACCEL;
    /** Maximum controlled acceleration (m/s^2). */
    public static double MAX_CONTROLLED_ACCEL;

    /** Default NetworkTables instance. */
    public static NetworkTableInstance NT_INSTANCE;
    /** Constant representing one million for time conversion. */
    public static int ONE_MILLION;
    /** Joystick deadband threshold. */
    public static double DEADBAND;

    /** CAN ID for the servo hub. */
    public static int SERVO_HUB_ID;
    /** The servo hub instance for controlling ratchet servos. */
    public static ServoHub SERVO_HUB;

    /** Constants for autonomous driving and path following. */
    public static final class AutoConstants {
        /** PathPlanner robot configuration loaded from GUI settings. */
        public static RobotConfig ROBOT_CONFIG;

        /** PathPlanner marker name for algae check. */
        public static String ALGAE_CHECK_MARKER;
        /** PathPlanner marker name for intake. */
        public static String INTAKE_MARKER;
        /** PathPlanner marker name for outtake. */
        public static String OUTTAKE_MARKER;

        /** Path constraints for path following. */
        public static PathConstraints PATH_CONSTRAINTS;

        /** Slant angle for object search rotation (degrees). */
        public static double OBJECT_SEARCH_DEGREE_SLANT;
        /** Degree tolerance for considering rotational alignment achieved. */
        public static double DEGREE_TOLERANCE_TO_ACCEPT;
        /** Translation tolerance for considering position achieved (meters). */
        public static double TRANSLATION_TOLERANCE_TO_ACCEPT;
        /** Translation tolerance for switching to direct drive (meters). */
        public static double TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE;
        /** Translation tolerance for drive mode transitions (meters). */
        public static double TRANSLATION_TOLERANCE_TO_TRANSITION;
        /** Translation tolerance for auto-mode transitions (meters). */
        public static double TRANSLATION_TOLERANCE_TO_TRANSITION_AUTO;
    }

    /** Constants for vision processing and camera mounts. */
    public static final class VisionConstants {
        /** Odometry standard deviation for pose estimation. */
        public static Matrix<N3, N1> ODOM_STD_DEV;
        /** Vision standard deviation function for multi-tag estimation based on distance. */
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION;
        /** Vision standard deviation function for single-tag estimation based on distance. */
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_FUNCTION;

        /** DIO port for the proximity sensor. */
        public static int PROXIMITY_SENSOR_DIO_PORT;

        /** Constants for the Limelight camera. */
        public static final class LimelightConstants {
            /** NetworkTables name for the Limelight. */
            public static String LIMELIGHT_NT_NAME;

            /** Forward mount offset of the Limelight (meters). */
            public static double LL_FORWARD;
            /** Right mount offset of the Limelight (meters). */
            public static double LL_RIGHT;
            /** Up mount offset of the Limelight (meters). */
            public static double LL_UP;
            /** Roll mount angle of the Limelight (degrees). */
            public static double LL_ROLL;
            /** Pitch mount angle of the Limelight (degrees). */
            public static double LL_PITCH;
            /** Yaw mount angle of the Limelight (degrees). */
            public static double LL_YAW;

            /** Maximum distance for tag clearance with the Limelight (meters). */
            public static double LL_MAX_TAG_CLEAR_DIST;
        }

        /** Constants for PhotonVision cameras. */
        public static final class PhotonConstants {
            /** Name of the color camera. */
            public static String COLOR_NAME;
            /** Forward mount offset of the color camera (meters). */
            public static double COLOR_FORWARD;
            /** Left mount offset of the color camera (meters). */
            public static double COLOR_LEFT;
            /** Up mount offset of the color camera (meters). */
            public static double COLOR_UP;
            /** Roll mount angle of the color camera (degrees). */
            public static double COLOR_ROLL;
            /** Pitch mount angle of the color camera (degrees). */
            public static double COLOR_PITCH;
            /** Yaw mount angle of the color camera (degrees). */
            public static double COLOR_YAW;

            /** Name of the top right black-and-white camera. */
            public static String BW_TOP_RIGHT_NAME;
            /** Forward mount offset of the top right BW camera (meters). */
            public static double BW_TOP_RIGHT_FORWARD;
            /** Left mount offset of the top right BW camera (meters). */
            public static double BW_TOP_RIGHT_LEFT;
            /** Up mount offset of the top right BW camera (meters). */
            public static double BW_TOP_RIGHT_UP;
            /** Roll mount angle of the top right BW camera (degrees). */
            public static double BW_TOP_RIGHT_ROLL;
            /** Pitch mount angle of the top right BW camera (degrees). */
            public static double BW_TOP_RIGHT_PITCH;
            /** Yaw mount angle of the top right BW camera (degrees). */
            public static double BW_TOP_RIGHT_YAW;

            /** Name of the top left black-and-white camera. */
            public static String BW_TOP_LEFT_NAME;
            /** Forward mount offset of the top left BW camera (meters). */
            public static double BW_TOP_LEFT_FORWARD;
            /** Left mount offset of the top left BW camera (meters). */
            public static double BW_TOP_LEFT_LEFT;
            /** Up mount offset of the top left BW camera (meters). */
            public static double BW_TOP_LEFT_UP;
            /** Roll mount angle of the top left BW camera (degrees). */
            public static double BW_TOP_LEFT_ROLL;
            /** Pitch mount angle of the top left BW camera (degrees). */
            public static double BW_TOP_LEFT_PITCH;
            /** Yaw mount angle of the top left BW camera (degrees). */
            public static double BW_TOP_LEFT_YAW;

            /** Name of the back black-and-white camera. */
            public static String BW_BACK_NAME;
            /** Forward mount offset of the back BW camera (meters). */
            public static double BW_BACK_FORWARD;
            /** Left mount offset of the back BW camera (meters). */
            public static double BW_BACK_LEFT;
            /** Up mount offset of the back BW camera (meters). */
            public static double BW_BACK_UP;
            /** Roll mount angle of the back BW camera (degrees). */
            public static double BW_BACK_ROLL;
            /** Pitch mount angle of the back BW camera (degrees). */
            public static double BW_BACK_PITCH;
            /** Yaw mount angle of the back BW camera (degrees). */
            public static double BW_BACK_YAW;

            /** Maximum distance for tag clearance with BW cameras (meters). */
            public static double BW_MAX_TAG_CLEAR_DIST;

            /** Target pitch angle for object detection (degrees). */
            public static double OBJECT_TARGET_PITCH;
        }

        /** Constants for the QuestNav headset-based localization. */
        public static final class QuestNavConstants {
            /** NetworkTables name for the QuestNav. */
            public static String QUESTNAV_NT_NAME;

            /** Forward mount offset of the QuestNav (meters). */
            public static double QUEST_FORWARD;
            /** Left mount offset of the QuestNav (meters). */
            public static double QUEST_LEFT;
            /** Up mount offset of the QuestNav (meters). */
            public static double QUEST_UP;
            /** Roll mount angle of the QuestNav (degrees). */
            public static double QUEST_ROLL;
            /** Pitch mount angle of the QuestNav (degrees). */
            public static double QUEST_PITCH;
            /** Yaw mount angle of the QuestNav (degrees). */
            public static double QUEST_YAW;

            // The thresholds through which the QuestNav's correctional offset will be recorrected by the error amount.
            /** Translation error tolerance before QuestNav correction is applied (meters). */
            public static double TRANSLATION_ERROR_TOLERANCE;
            /** Rotation error tolerance before QuestNav correction is applied (degrees). */
            public static double ROTATION_ERROR_TOLERANCE;

            /** Minimum distance to consider a tag as far for QuestNav (meters). */
            public static double MIN_TAG_DIST_TO_BE_FAR;
        }
    }

    /** Constants for the elevator mechanism. */
    public final static class ElevatorConstants {
        // motor config
        /** CAN ID for the elevator leader motor. */
        public static int LEAD_ID;
        /** CAN ID for the elevator follower motor. */
        public static int FOLLOWER_ID;
        /** DIO port for the elevator lower limit switch. */
        public static int LOWER_LIMIT_SWITCH_DIO_PORT;
        /** Current limit for elevator motors (amps). */
        public static double CURRENT_LIMIT;
        /** Motor invert setting for elevator motors. */
        public static InvertedValue MOTOR_INVERT;
        /** Neutral mode for elevator motors. */
        public static NeutralModeValue NEUTRAL_MODE;

        // mechanism characterization
        /** Ratio of rotor rotations to inches of elevator travel. */
        public static double ROTOR_TO_INCH_RATIO;
        /** Total mass of the elevator carriage (lbs). */
        public static double MASS_LBS;
        /** Center of mass ratio for stage 2 extension. */
        public static double COM_TO_STAGE_2_RATIO;
        /** Stage 3 extension limit. */
        public static double STAGE_3_LIMIT;
        /** Center of mass ratio for stage 3 extension. */
        public static double COM_TO_STAGE_3_RATIO;
        /** Center of mass translation at zero upright position. */
        public static Translation2d ZERO_UPRIGHT_COM;

        // pid & tolerance
        /** Gravity feedforward function dependent on pivot angle (volts). */
        public static Function<Double, Double> G;
        /** Velocity feedforward gain (V / (rotor rps)). */
        public static double V;
        /** Acceleration feedforward gain (V / (rotor rps^2)). */
        public static double A;
        /** Proportional gain for the elevator PID controller. */
        public static double P;
        /** Integral gain for the elevator PID controller. */
        public static double I;
        /** Derivative gain for the elevator PID controller. */
        public static double D;
        /** Exponential velocity feedforward at reduced voltage allocation. */
        public static double EXPO_V;
        /** Exponential acceleration feedforward at reduced voltage allocation. */
        public static double EXPO_A;
        /** Safe tolerance for elevator position (inches). */
        public static double SAFE_TOLERANCE;
        /** Tolerance threshold for considering the elevator at target (inches). */
        public static double AT_TARGET_TOLERANCE;

        // presets
        /** Lower physical limit of elevator travel (inches). */
        public static double LOWER_LIMIT;
        /** Upper physical limit of elevator travel (inches). */
        public static double UPPER_LIMIT;
        /** Stow position (inches). */
        public static double STOW;
        /** Stow position for L2/L3/L4 scoring (inches). */
        public static double L2_L3_L4_STOW;
        /** Coral station intake position (inches). */
        public static double CORAL_STATION;
        /** Coral station intake position when obstructed (inches). */
        public static double CORAL_STATION_OBSTRUCTED;
        /** Ground coral intake position (inches). */
        public static double GROUND_CORAL;
        /** Ground algae intake position (inches). */
        public static double GROUND_ALGAE;
        /** L1 coral scoring position (inches). */
        public static double L1_CORAL;
        /** L2 coral position at branch (inches). */
        public static double L2_CORAL_AT_BRANCH;
        /** L2 coral position one coral from branch (inches). */
        public static double L2_CORAL_ONE_CORAL_FROM_BRANCH;
        /** L3 coral position at branch (inches). */
        public static double L3_CORAL_AT_BRANCH;
        /** L3 coral position one coral from branch (inches). */
        public static double L3_CORAL_ONE_CORAL_FROM_BRANCH;
        /** L4 coral position at branch (inches). */
        public static double L4_CORAL_AT_BRANCH;
        /** L4 coral position one coral from branch (inches). */
        public static double L4_CORAL_ONE_CORAL_FROM_BRANCH;
        /** Low reef algae removal position (inches). */
        public static double LOW_REEF_ALGAE;
        /** High reef algae removal position (inches). */
        public static double HIGH_REEF_ALGAE;
        /** Processor scoring position (inches). */
        public static double PROCESSOR;
        /** Net scoring position (inches). */
        public static double NET;
        /** Prepare climb position (inches). */
        public static double PREPARE_CLIMB;
        /** Climb position (inches). */
        public static double CLIMB;
    }

    /** Constants for the intake mechanism. */
    public final static class IntakeConstants {
        // basic configs
        /** CAN ID for the intake motor. */
        public static int MOTOR_ID;
        /** CAN ID for the intake sensor. */
        public static int SENSOR_ID;
        /** DIO port for the intake beam break sensor. */
        public static int BEAMBREAK_DIO_PORT;
        /** Current limit for the intake motor (amps). */
        public static double CURRENT_LIMIT;
        /** Motor invert setting for the intake. */
        public static InvertedValue MOTOR_INVERT;
        /** Neutral mode for the intake motor. */
        public static NeutralModeValue NEUTRAL_MODE;
        /** Default proximity threshold for object detection. */
        public static double DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD;
    }

    /** Constants for the pivot mechanism. */
    public final static class PivotConstants {
        // motor config
        /** CAN ID for the pivot leader motor. */
        public static int LEAD_ID;
        /** CAN ID for the pivot follower motor. */
        public static int FOLLOWER_ID;
        /** CAN ID for the pivot intake motor. */
        public static int INTAKE_ID;
        /** Current limit for pivot motors (amps). */
        public static double CURRENT_LIMIT;
        /** Motor invert setting for pivot motors. */
        public static InvertedValue MOTOR_INVERT;
        /** Motor invert setting for the pivot intake motor. */
        public static InvertedValue INTAKE_MOTOR_INVERT;
        /** Neutral mode for pivot motors. */
        public static NeutralModeValue NEUTRAL_MODE;

        // mechanism characterization
        /** Ratio of sensor units to degrees of pivot rotation. */
        public static double SENSOR_TO_DEGREE_RATIO;
        /** Position of the pivot axis relative to robot center (inches). */
        public static Translation2d AXIS_POSITION;

        // encoder config
        /** CAN ID for the pivot CANcoder. */
        public static int ENCODER_ID;
        /** Absolute encoder offset for the pivot CANcoder (rotations). */
        public static double ENCODER_ABSOLUTE_OFFSET;
        /** Encoder invert direction for the pivot. */
        public static SensorDirectionValue ENCODER_INVERT;

        // up ratchet config
        /** Servo channel for the up ratchet. */
        public static ServoChannel.ChannelId UP_RATCHET_CHANNEL;
        /** Servo pulse width for engaging the up ratchet (microseconds). */
        public static int UP_RATCHET_ON;
        /** Servo pulse width for disengaging the up ratchet (microseconds). */
        public static int UP_RATCHET_OFF;

        // down ratchet config
        /** Servo channel for the down ratchet. */
        public static ServoChannel.ChannelId DOWN_RATCHET_CHANNEL;
        /** Servo pulse width for engaging the down ratchet (microseconds). */
        public static int DOWN_RATCHET_ON;
        /** Servo pulse width for disengaging the down ratchet (microseconds). */
        public static int DOWN_RATCHET_OFF;

        // pid & tolerance
        /** Gravity feedforward gain for the pivot (volts). */
        public static double G;
        /** Velocity feedforward gain (V / (rotor rps)). */
        public static double V;
        /** Acceleration feedforward gain (V / (rotor rps^2)). */
        public static double A;
        /** Proportional gain for the pivot PID controller. */
        public static double P;
        /** Integral gain for the pivot PID controller. */
        public static double I;
        /** Derivative gain for the pivot PID controller. */
        public static double D;
        /** Exponential velocity feedforward at reduced voltage allocation. */
        public static double EXPO_V;
        /** Exponential acceleration feedforward at reduced voltage allocation. */
        public static double EXPO_A;
        /** Exponential slow velocity feedforward for precision moves. */
        public static double EXPO_V_SLOW;
        /** Safe tolerance for pivot position (degrees). */
        public static double SAFE_TOLERANCE;
        /** Tolerance threshold for considering the pivot at target (degrees). */
        public static double AT_TARGET_TOLERANCE;

        // presets
        /** Lower physical limit of pivot travel (degrees). */
        public static double LOWER_LIMIT;
        /** Upper physical limit of pivot travel (degrees). */
        public static double UPPER_LIMIT;
        /** Stow position (degrees). */
        public static double STOW;
        /** Stow position for L2/L3/L4 scoring (degrees). */
        public static double L2_L3_L4_STOW;
        /** Coral station intake position (degrees). */
        public static double CORAL_STATION;
        /** Coral station intake position when obstructed (degrees). */
        public static double CORAL_STATION_OBSTRUCTED;
        /** Ground coral intake position (degrees). */
        public static double GROUND_CORAL;
        /** Ground algae intake position (degrees). */
        public static double GROUND_ALGAE;
        /** L1 coral scoring position (degrees). */
        public static double L1_CORAL;
        /** L2 coral position at branch (degrees). */
        public static double L2_CORAL_AT_BRANCH;
        /** L2 coral position one coral from branch (degrees). */
        public static double L2_CORAL_ONE_CORAL_FROM_BRANCH;
        /** L3 coral position at branch (degrees). */
        public static double L3_CORAL_AT_BRANCH;
        /** L3 coral position one coral from branch (degrees). */
        public static double L3_CORAL_ONE_CORAL_FROM_BRANCH;
        /** L4 coral position at branch (degrees). */
        public static double L4_CORAL_AT_BRANCH;
        /** L4 coral position one coral from branch (degrees). */
        public static double L4_CORAL_ONE_CORAL_FROM_BRANCH;
        /** Low reef algae removal position (degrees). */
        public static double LOW_REEF_ALGAE;
        /** High reef algae removal position (degrees). */
        public static double HIGH_REEF_ALGAE;
        /** Processor scoring position (degrees). */
        public static double PROCESSOR;
        /** Net scoring position (degrees). */
        public static double NET;
        /** Prepare climb position (degrees). */
        public static double PREPARE_CLIMB;
        /** Climb position (degrees). */
        public static double CLIMB;
    }

    /** Constants for the wrist mechanism. */
    public final static class WristConstants {
        // motor config
        /** CAN ID for the wrist motor. */
        public static int MOTOR_ID;
        /** Current limit for the wrist motor (amps). */
        public static double CURRENT_LIMIT;
        /** Motor invert setting for the wrist motor. */
        public static InvertedValue MOTOR_INVERT;
        /** Neutral mode for the wrist motor. */
        public static NeutralModeValue NEUTRAL_MODE;

        // mechanism characterization
        /** Ratio of sensor units to degrees of wrist rotation. */
        public static double SENSOR_TO_DEGREE_RATIO;
        /** Total mass of the wrist (lbs). */
        public static double MASS_LBS;
        /** Position of the wrist axis relative to robot center (inches). */
        public static Translation2d AXIS_POSITION;
        /** Center of mass offset from the wrist axis at zero position (inches). */
        public static Translation2d AXIS_TO_ZERO_COM;

        // encoder config
        /** CAN ID for the wrist CANcoder. */
        public static int ENCODER_ID;
        /** Absolute encoder offset for the wrist CANcoder (rotations). */
        public static double ENCODER_ABSOLUTE_OFFSET;
        /** Encoder invert direction for the wrist. */
        public static SensorDirectionValue ENCODER_INVERT;

        // pid & tolerance
        /** Gravity feedforward function dependent on wrist and pivot angles (volts). */
        public static BiFunction<Double, Double, Double> G;
        /** Velocity feedforward gain (V / (rotor rps)). */
        public static double V;
        /** Acceleration feedforward gain (V / (rotor rps^2)). */
        public static double A;
        /** Proportional gain for the wrist PID controller. */
        public static double P;
        /** Integral gain for the wrist PID controller. */
        public static double I;
        /** Derivative gain for the wrist PID controller. */
        public static double D;
        /** Exponential velocity feedforward at reduced voltage allocation. */
        public static double EXPO_V;
        /** Exponential acceleration feedforward at reduced voltage allocation. */
        public static double EXPO_A;
        /** Safe tolerance for wrist position (degrees). */
        public static double SAFE_TOLERANCE;
        /** Tolerance threshold for considering the wrist at target (degrees). */
        public static double AT_TARGET_TOLERANCE;

        // presets
        /** Lower position limit function dependent on elevator and pivot positions (degrees). */
        public static BiFunction<Double, Double, Double> LOWER_LIMIT;
        /** Upper position limit function dependent on elevator position (degrees). */
        public static Function<Double, Double> UPPER_LIMIT;
        /** Stow position (degrees). */
        public static double STOW;
        /** Stow position for L2/L3/L4 scoring (degrees). */
        public static double L2_L3_L4_STOW;
        /** Coral station intake position (degrees). */
        public static double CORAL_STATION;
        /** Coral station intake position when obstructed (degrees). */
        public static double CORAL_STATION_OBSTRUCTED;
        /** Ground coral intake position (degrees). */
        public static double GROUND_CORAL;
        /** Ground algae intake position (degrees). */
        public static double GROUND_ALGAE;
        /** L1 coral scoring position (degrees). */
        public static double L1_CORAL;
        /** L2 coral position at branch (degrees). */
        public static double L2_CORAL_AT_BRANCH;
        /** L2 coral position one coral from branch (degrees). */
        public static double L2_CORAL_ONE_CORAL_FROM_BRANCH;
        /** L3 coral position at branch (degrees). */
        public static double L3_CORAL_AT_BRANCH;
        /** L3 coral position one coral from branch (degrees). */
        public static double L3_CORAL_ONE_CORAL_FROM_BRANCH;
        /** L4 coral position at branch (degrees). */
        public static double L4_CORAL_AT_BRANCH;
        /** L4 coral position one coral from branch (degrees). */
        public static double L4_CORAL_ONE_CORAL_FROM_BRANCH;
        /** Low reef algae removal position (degrees). */
        public static double LOW_REEF_ALGAE;
        /** High reef algae removal position (degrees). */
        public static double HIGH_REEF_ALGAE;
        /** Processor scoring position (degrees). */
        public static double PROCESSOR;
        /** Net scoring position (degrees). */
        public static double NET;
        /** Prepare climb position (degrees). */
        public static double PREPARE_CLIMB;
        /** Climb position (degrees). */
        public static double CLIMB;
    }

    /** Constants for swerve drivetrain PID gains and module configurations. */
    public static final class SwerveConstants {
        /** Path translation PID proportional gain for PathPlanner (FOR PATHPLANNER AND TRANSLATIONAL CONTROL). */
        public static double PATH_TRANSLATION_CONTROLLER_P;
        /** Path translation PID derivative gain for PathPlanner. */
        public static double PATH_TRANSLATION_CONTROLLER_D;
        /** Path rotation PID proportional gain for PathPlanner. */
        public static double PATH_ROTATION_CONTROLLER_P;

        /** Angular position PID proportional gain for yaw control (FOR ANGULAR ROTATION CONTROL). */
        public static double ANGULAR_POSITION_P;
        /** Angular position PID derivative gain for yaw control. */
        public static double ANGULAR_POSITION_D;

        /** Angular object detection PID proportional gain (FOR OBJECT DETECTION ROTATION CONTROL). */
        public static double ANGULAR_OBJECT_DETECTION_P;
        /** Angular object detection PID derivative gain. */
        public static double ANGULAR_OBJECT_DETECTION_D;

        /** Minimum angle for continuous angular input (-180 degrees). */
        public static double ANGULAR_MINIMUM_ANGLE;
        /** Maximum angle for continuous angular input (180 degrees). */
        public static double ANGULAR_MAXIMUM_ANGLE;

        /** Stator current limit threshold for slip detection (amps). */
        public static Current SLIP_CURRENT;
        /** Audio configuration for the swerve drivetrain. */
        public static AudioConfigs AUDIO_CONFIGS;

        /** Swerve drivetrain constants including CAN bus and Pigeon 2 IMU. */
        public static SwerveDrivetrainConstants SWERVE_DRIVETRAIN_CONSTANTS;

        /** Front left swerve module constants. */
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT;
        /** Front right swerve module constants. */
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT;
        /** Back left swerve module constants. */
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT;
        /** Back right swerve module constants. */
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT;
    }
}