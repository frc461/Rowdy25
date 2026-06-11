package io.github.frc461.rowdy25.constants.variants;

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

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import io.github.frc461.rowdy25.util.FieldUtil;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Default robot constant values used as the baseline for all robot identities.
 * <p>
 * These provide standard component IDs, mechanism characterization constants,
 * PID/feedforward gains, motion constraints, vision camera mount offsets, and
 * swerve module configurations. Robot-specific overrides are applied on top of
 * these defaults in {@link io.github.frc461.rowdy25.constants.RobotIdentity#initializeConstants()}.
 *
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 * @author Mysterious Man, <a href="https://github.com/tidymoman">GitHub</a>
 */
public final class DefaultConstants {

    // CAN bus that the devices are located on;
    // If there is more than one CAN bus, create a CANBus constant for each one
    /** The CAN bus used by the drivetrain and subsystem devices. */
    public static final CANBus CAN_BUS = new CANBus("", "./logs/example.hoot");

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    /** Default rotation for the blue alliance (facing the field). */
    public static final Rotation2d BLUE_DEFAULT_ROTATION = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    /** Default rotation for the red alliance (facing the field). */
    public static final Rotation2d RED_DEFAULT_ROTATION = Rotation2d.fromDegrees(180);

    /** Length of the robot chassis including bumpers. */
    public static final Distance ROBOT_LENGTH_WITH_BUMPERS = Inches.of(38.5);
    /** Width of the robot chassis including bumpers. */
    public static final Distance ROBOT_WIDTH_WITH_BUMPERS = Inches.of(32.5);

    /** Supplier for the current DriverStation alliance. */
    public static final Supplier<DriverStation.Alliance> ALLIANCE_SUPPLIER = () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    /** Computes the center-left coral station scoring pose based on alliance. */
    public static final Function<Supplier<DriverStation.Alliance>, Pose2d> CENTER_OF_LEFT_CORAL_STATION = allianceSupplier ->
            allianceSupplier.get() == DriverStation.Alliance.Red
                    ? FieldUtil.AprilTag.ID_1.pose2d
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero))
                    : FieldUtil.AprilTag.ID_13.pose2d
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero));

    /** Computes the center-right coral station scoring pose based on alliance. */
    public static final Function<Supplier<DriverStation.Alliance>, Pose2d> CENTER_OF_RIGHT_CORAL_STATION = allianceSupplier ->
            allianceSupplier.get() == DriverStation.Alliance.Red
                    ? FieldUtil.AprilTag.ID_2.pose2d
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero))
                    : FieldUtil.AprilTag.ID_12.pose2d
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero));

    /** Computes the far-left coral station scoring pose based on alliance. */
    public static final Function<Supplier<DriverStation.Alliance>, Pose2d> FAR_LEFT_CORAL_STATION = allianceSupplier ->
            allianceSupplier.get() == DriverStation.Alliance.Red
                    ? new Pose2d(Units.inchesToMeters(623.07), Units.inchesToMeters(0), FieldUtil.AprilTag.ID_1.pose2d.getRotation())
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), ROBOT_WIDTH_WITH_BUMPERS.div(2).unaryMinus().in(Meters), Rotation2d.kZero))
                    : new Pose2d(Units.inchesToMeters(67.82), Units.inchesToMeters(316.63), FieldUtil.AprilTag.ID_13.pose2d.getRotation())
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), ROBOT_WIDTH_WITH_BUMPERS.div(2).unaryMinus().in(Meters), Rotation2d.kZero));
    /** Computes the far-right coral station scoring pose based on alliance. */
    public static final Function<Supplier<DriverStation.Alliance>, Pose2d> FAR_RIGHT_CORAL_STATION = allianceSupplier ->
            allianceSupplier.get() == DriverStation.Alliance.Red
                    ? new Pose2d(Units.inchesToMeters(623.07), Units.inchesToMeters(316.63), FieldUtil.AprilTag.ID_2.pose2d.getRotation())
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), ROBOT_WIDTH_WITH_BUMPERS.div(2).in(Meters), Rotation2d.kZero))
                    : new Pose2d(Units.inchesToMeters(67.82), Units.inchesToMeters(0), FieldUtil.AprilTag.ID_12.pose2d.getRotation())
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), ROBOT_WIDTH_WITH_BUMPERS.div(2).in(Meters), Rotation2d.kZero));

    // kSpeedAt12Volts desired top speed
    /** Maximum translational velocity at 12V (m/s). */
    public static final double MAX_VEL = SwerveConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);
    /** Maximum controlled velocity as a function of elevator height (m/s). */
    public static final Function<Double, Double> MAX_CONTROLLED_VEL = elevatorHeight -> MAX_VEL - 0.09 * elevatorHeight;
    // 1.96664381049 rotations per second tuned max angular velocity
    /** Maximum controlled angular velocity as a function of elevator height (rad/s). */
    public static final Function<Double, Double> MAX_CONTROLLED_ANGULAR_VEL = elevatorHeight -> RotationsPerSecond.of(0.75).in(RadiansPerSecond) - 0.07 * elevatorHeight;
    /** Maximum angular velocity as a function of elevator height (rad/s). */
    public static final Function<Double, Double> MAX_ANGULAR_VEL = elevatorHeight -> elevatorHeight < 16 ? RotationsPerSecond.of(1.96664381049).in(RadiansPerSecond) : MAX_CONTROLLED_ANGULAR_VEL.apply(elevatorHeight);

    /** Maximum translational acceleration (m/s^2). */
    public static final double MAX_ACCEL = MetersPerSecondPerSecond.of(10.8).in(MetersPerSecondPerSecond);
    /** Maximum angular acceleration (rad/s^2). */
    public static final double MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(485.0).in(RadiansPerSecondPerSecond);
    /** Maximum controlled acceleration (m/s^2). */
    public static final double MAX_CONTROLLED_ACCEL = MetersPerSecondPerSecond.of(5.0).in(MetersPerSecondPerSecond);

    /** Default NetworkTables instance. */
    public static final NetworkTableInstance NT_INSTANCE = NetworkTableInstance.getDefault();
    /** Constant representing one million for time conversion. */
    public static final int ONE_MILLION = 1_000_000;
    /** Joystick deadband threshold. */
    public static final double DEADBAND = 0.1;

    /** CAN ID for the servo hub. */
    public static final int SERVO_HUB_ID = 54;
    /** The servo hub instance for controlling ratchet servos. */
    public static final ServoHub SERVO_HUB = new ServoHub(SERVO_HUB_ID);

    /** Constants for autonomous driving and path following. */
    public static final class AutoConstants {
        /** PathPlanner robot configuration loaded from GUI settings. */
        public static final RobotConfig ROBOT_CONFIG;

        static {
            try {
                ROBOT_CONFIG = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
        }

        /** PathPlanner marker name for algae check. */
        public static final String ALGAE_CHECK_MARKER = "checkAlgae";
        /** PathPlanner marker name for intake. */
        public static final String INTAKE_MARKER = "intake";
        /** PathPlanner marker name for outtake. */
        public static final String OUTTAKE_MARKER = "outtake";

        /** Path constraints for path following. */
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                MAX_VEL,
                MAX_CONTROLLED_ACCEL,
                MAX_CONTROLLED_ANGULAR_VEL.apply(0.0),
                MAX_ANGULAR_ACCEL
        );

        /** Slant angle for object search rotation (degrees). */
        public static final double OBJECT_SEARCH_DEGREE_SLANT = 30.0;
        /** Degree tolerance for considering rotational alignment achieved. */
        public static final double DEGREE_TOLERANCE_TO_ACCEPT = 2.5;
        /** Translation tolerance for considering position achieved (meters). */
        public static final double TRANSLATION_TOLERANCE_TO_ACCEPT = 0.03;
        /** Translation tolerance for switching to direct drive (meters). */
        public static final double TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE = 0.5;
        /** Translation tolerance for drive mode transitions (meters). */
        public static final double TRANSLATION_TOLERANCE_TO_TRANSITION = 1.8;
        /** Translation tolerance for auto-mode transitions (meters). */
        public static final double TRANSLATION_TOLERANCE_TO_TRANSITION_AUTO = 1.5;
    }

    /** Constants for vision processing and camera mounts. */
    public static final class VisionConstants {
        /** Odometry standard deviation for pose estimation. */
        public static final Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(0.01));
        /** Vision standard deviation function for multi-tag estimation based on distance. */
        public static final Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION =
                dist -> dist < 3.0
                        ? VecBuilder.fill(Math.min(0.03, 0.03 * dist), Math.min(0.03, 0.03 * dist), DriverStation.isEnabled() ? Units.degreesToRadians(5.0) : Units.degreesToRadians(0.05))
                        : VecBuilder.fill(0.05 * dist, 0.05 * dist, Units.degreesToRadians(180.0) * dist);
        /** Vision standard deviation function for single-tag estimation based on distance. */
        public static final Function<Double, Matrix<N3, N1>> VISION_STD_DEV_FUNCTION =
                dist -> dist < 3.0
                        ? VecBuilder.fill(0.075 * dist, 0.075 * dist, Units.degreesToRadians(180.0) * dist)
                        : VecBuilder.fill(0.1 * dist, 0.1 * dist, Units.degreesToRadians(180.0) * dist);

        /** DIO port for the proximity sensor. */
        public static final int PROXIMITY_SENSOR_DIO_PORT = 5;

        /** Constants for the Limelight camera. */
        public static final class LimelightConstants {
            /** NetworkTables name for the Limelight. */
            public static final String LIMELIGHT_NT_NAME = "limelight";

            /** Forward mount offset of the Limelight (meters). */
            public static final double LL_FORWARD = 0.0;
            /** Right mount offset of the Limelight (meters). */
            public static final double LL_RIGHT = 0.0;
            /** Up mount offset of the Limelight (meters). */
            public static final double LL_UP = 0.0;
            /** Roll mount angle of the Limelight (degrees). */
            public static final double LL_ROLL = 0.0;
            /** Pitch mount angle of the Limelight (degrees). */
            public static final double LL_PITCH = 0.0;
            /** Yaw mount angle of the Limelight (degrees). */
            public static final double LL_YAW = 0.0;

            /** Maximum distance for tag clearance with the Limelight (meters). */
            public static final double LL_MAX_TAG_CLEAR_DIST = 4.0;
        }

        /** Constants for PhotonVision cameras. */
        public static final class PhotonConstants {
            /** Name of the color camera. */
            public static final String COLOR_NAME = "ArducamColor";
            /** Forward mount offset of the color camera (meters). */
            public static final double COLOR_FORWARD = -0.314091;
            /** Left mount offset of the color camera (meters). */
            public static final double COLOR_LEFT = -0.259556;
            /** Up mount offset of the color camera (meters). */
            public static final double COLOR_UP = 0.184150;
            /** Roll mount angle of the color camera (degrees). */
            public static final double COLOR_ROLL = 0.0;
            /** Pitch mount angle of the color camera (degrees). */
            public static final double COLOR_PITCH = 0.0;
            /** Yaw mount angle of the color camera (degrees). */
            public static final double COLOR_YAW = 150.0;

            /** Name of the top right black-and-white camera. */
            public static final String BW_TOP_RIGHT_NAME = "ArducamBW2";
            /** Forward mount offset of the top right BW camera (meters). */
            public static final double BW_TOP_RIGHT_FORWARD = 0.404;
            /** Left mount offset of the top right BW camera (meters). */
            public static final double BW_TOP_RIGHT_LEFT = -0.291321;
            /** Up mount offset of the top right BW camera (meters). */
            public static final double BW_TOP_RIGHT_UP = 0.279631;
            /** Roll mount angle of the top right BW camera (degrees). */
            public static final double BW_TOP_RIGHT_ROLL = 0.0;
            /** Pitch mount angle of the top right BW camera (degrees). */
            public static final double BW_TOP_RIGHT_PITCH = -5.0;
            /** Yaw mount angle of the top right BW camera (degrees). */
            public static final double BW_TOP_RIGHT_YAW = -30.0;

            /** Name of the top left black-and-white camera. */
            public static final String BW_TOP_LEFT_NAME = "ArducamBW";
            /** Forward mount offset of the top left BW camera (meters). */
            public static final double BW_TOP_LEFT_FORWARD = 0.404;
            /** Left mount offset of the top left BW camera (meters). */
            public static final double BW_TOP_LEFT_LEFT = 0.291321;
            /** Up mount offset of the top left BW camera (meters). */
            public static final double BW_TOP_LEFT_UP = 0.279631;
            /** Roll mount angle of the top left BW camera (degrees). */
            public static final double BW_TOP_LEFT_ROLL = 0.0;
            /** Pitch mount angle of the top left BW camera (degrees). */
            public static final double BW_TOP_LEFT_PITCH = -5.0;
            /** Yaw mount angle of the top left BW camera (degrees). */
            public static final double BW_TOP_LEFT_YAW = 30.0;

            /** Name of the back black-and-white camera. */
            public static final String BW_BACK_NAME = "ArducamBW3";
            /** Forward mount offset of the back BW camera (meters). */
            public static final double BW_BACK_FORWARD = -0.315691;
            /** Left mount offset of the back BW camera (meters). */
            public static final double BW_BACK_LEFT = 0.266709;
            /** Up mount offset of the back BW camera (meters). */
            public static final double BW_BACK_UP = 0.186127;
            /** Roll mount angle of the back BW camera (degrees). */
            public static final double BW_BACK_ROLL = 0.0;
            /** Pitch mount angle of the back BW camera (degrees). */
            public static final double BW_BACK_PITCH = -8.0;
            /** Yaw mount angle of the back BW camera (degrees). */
            public static final double BW_BACK_YAW = 180;

            /** Maximum distance for tag clearance with BW cameras (meters). */
            public static final double BW_MAX_TAG_CLEAR_DIST = 5.5;

            /** Target pitch angle for object detection (degrees). */
            public static final double OBJECT_TARGET_PITCH = -15;
        }

        /** Constants for the QuestNav headset-based localization. */
        public static final class QuestNavConstants {
            /** NetworkTables name for the QuestNav. */
            public static final String QUESTNAV_NT_NAME = "questnav";

            // TODO WAIT (NEXT QUEST NAV UPDATE): SET QUEST TO CENTER OF ROBOT OFFSETS
            /** Forward mount offset of the QuestNav (meters). */
            public static final double QUEST_FORWARD = Units.inchesToMeters(-2.5);
            /** Left mount offset of the QuestNav (meters). */
            public static final double QUEST_LEFT = Units.inchesToMeters(5.25);
            /** Up mount offset of the QuestNav (meters). */
            public static final double QUEST_UP = 0.0;
            /** Roll mount angle of the QuestNav (degrees). */
            public static final double QUEST_ROLL = 0.0;
            /** Pitch mount angle of the QuestNav (degrees). */
            public static final double QUEST_PITCH = 0.0;
            /** Yaw mount angle of the QuestNav (degrees). */
            public static final double QUEST_YAW = 0.0;

            // The error threshold to cross when QuestNav's correctional offset will be re-corrected by the error amount.
            /** Translation error tolerance before QuestNav correction is applied (meters). */
            public static final double TRANSLATION_ERROR_TOLERANCE = 0.1;
            /** Rotation error tolerance before QuestNav correction is applied (degrees). */
            public static final double ROTATION_ERROR_TOLERANCE = 3.0;

            /** Minimum distance to consider a tag as far for QuestNav (meters). */
            public static final double MIN_TAG_DIST_TO_BE_FAR = 5.0;
        }
    }

    /** Constants for the elevator mechanism. */
    public final static class ElevatorConstants {
        // motor config
        /** CAN ID for the elevator leader motor. */
        public static final int LEAD_ID = 31;
        /** CAN ID for the elevator follower motor. */
        public static final int FOLLOWER_ID = 32;
        /** DIO port for the elevator lower limit switch. */
        public static final int LOWER_LIMIT_SWITCH_DIO_PORT = 0;
        /** Current limit for elevator motors (amps). */
        public static final double CURRENT_LIMIT = 40;
        /** Motor invert setting for elevator motors. */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        /** Neutral mode for elevator motors (coast). */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

        // mechanism characterization
        private static final double ROTOR_TO_PULLEY_RATIO = 11.57;
        private static final double PULLEY_CIRCUMFERENCE = 7.065;
        /** Ratio of rotor rotations to inches of elevator travel. */
        public static final double ROTOR_TO_INCH_RATIO = ROTOR_TO_PULLEY_RATIO / PULLEY_CIRCUMFERENCE;
        private static final double STAGE_2_LOAD_LBS = 28.44;
        /** Total mass of the elevator carriage (lbs). */
        public static final double MASS_LBS = 23.0132625;
        /** Center of mass ratio for stage 2 extension. */
        public static final double COM_TO_STAGE_2_RATIO = 0.509767;
        /** Stage 3 extension limit. */
        public static final double STAGE_3_LIMIT = 22;
        /** Center of mass ratio for stage 3 extension. */
        public static final double COM_TO_STAGE_3_RATIO = 0.3345002;
        /** Center of mass translation at zero upright position. */
        public static final Translation2d ZERO_UPRIGHT_COM = new Translation2d(-11.347053, 15.125012);

        // pid & tolerance
        /** Gravity feedforward function dependent on pivot angle (volts). */
        public static final Function<Double, Double> G = (pivotDeg) -> 0.2175 * Math.sin(Math.toRadians(pivotDeg));
        /** Velocity feedforward gain (V / (rotor rps)). */
        public static final double V = 0.31 / ROTOR_TO_INCH_RATIO; // 1V / (in/s) -> 1V / (rotor rps)
        /** Acceleration feedforward gain (V / (rotor rps^2)). */
        public static final double A = 0.001 / ROTOR_TO_INCH_RATIO; // 1V / (in/s^2) -> 1V / (rotor rps^2)
        /** Proportional gain for the elevator PID controller. */
        public static final double P = 0.25;
        /** Integral gain for the elevator PID controller. */
        public static final double I = 0.0;
        /** Derivative gain for the elevator PID controller. */
        public static final double D = 0.025;
        /** Exponential velocity feedforward at reduced voltage allocation. */
        public static final double EXPO_V = V / 0.90; // 90% of the actual max velocity, as it will allocate 1 / 0.9 = 1.1111 times the voltage to 1 rps
        /** Exponential acceleration feedforward at reduced voltage allocation. */
        public static final double EXPO_A = A / 0.015; // 1.5% of the actual max accel
        /** Safe tolerance for elevator position (inches). */
        public static final double SAFE_TOLERANCE = 15.0;
        /** Tolerance threshold for considering the elevator at target (inches). */
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        /** Lower physical limit of elevator travel (inches). */
        public static final double LOWER_LIMIT = 0;
        /** Upper physical limit of elevator travel (inches). */
        public static final double UPPER_LIMIT = 46;
        /** Stow position (inches). */
        public static final double STOW = 0;
        /** Stow position for L2/L3/L4 scoring (inches). */
        public static final double L2_L3_L4_STOW = 5.5;
        /** Coral station intake position (inches). */
        public static final double CORAL_STATION = 0;
        /** Coral station intake position when obstructed (inches). */
        public static final double CORAL_STATION_OBSTRUCTED = 0;
        /** Ground coral intake position (inches). */
        public static final double GROUND_CORAL = 0;
        /** Ground algae intake position (inches). */
        public static final double GROUND_ALGAE = 0;
        /** L1 coral scoring position (inches). */
        public static final double L1_CORAL = 0;
        /** L2 coral position at branch (inches). */
        public static final double L2_CORAL_AT_BRANCH = 0;
        /** L2 coral position one coral from branch (inches). */
        public static final double L2_CORAL_ONE_CORAL_FROM_BRANCH = 0;
        /** L3 coral position at branch (inches). */
        public static final double L3_CORAL_AT_BRANCH = 17.2;
        /** L3 coral position one coral from branch (inches). */
        public static final double L3_CORAL_ONE_CORAL_FROM_BRANCH = 17.2;
        /** L4 coral position at branch (inches). */
        public static final double L4_CORAL_AT_BRANCH = 41.5;
        /** L4 coral position one coral from branch (inches). */
        public static final double L4_CORAL_ONE_CORAL_FROM_BRANCH = 41.5;
        /** Low reef algae removal position (inches). */
        public static final double LOW_REEF_ALGAE = 2.0;
        /** High reef algae removal position (inches). */
        public static final double HIGH_REEF_ALGAE = 3.0;
        /** Processor scoring position (inches). */
        public static final double PROCESSOR = 0;
        /** Net scoring position (inches). */
        public static final double NET = 44;
        /** Prepare climb position (inches). */
        public static final double PREPARE_CLIMB = 0;
        /** Climb position (inches). */
        public static final double CLIMB = 5.5;
    }

    /** Constants for the intake mechanism. */
    public final static class IntakeConstants {
        /** CAN ID for the intake motor. */
        public static final int MOTOR_ID = 41;
        /** CAN ID for the intake sensor. */
        public static final int SENSOR_ID = 42;
        /** DIO port for the intake beam break sensor. */
        public static final int BEAMBREAK_DIO_PORT = 4;
        /** Current limit for the intake motor (amps). */
        public static final double CURRENT_LIMIT = 40;
        /** Motor invert setting for the intake. */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        /** Neutral mode for the intake motor (coast). */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        /** Default proximity threshold for object detection. */
        public static final double DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD = 0.1;
    }

    /** Constants for the pivot mechanism. */
    public final static class PivotConstants {
        // motor config
        /** CAN ID for the pivot leader motor. */
        public static final int LEAD_ID = 51;
        /** CAN ID for the pivot follower motor. */
        public static final int FOLLOWER_ID = 52;
        /** CAN ID for the pivot intake motor. */
        public static final int INTAKE_ID = 55;
        /** Current limit for pivot motors (amps). */
        public static final double CURRENT_LIMIT = 40;
        /** Motor invert setting for pivot motors. */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        /** Motor invert setting for the pivot intake motor. */
        public static final InvertedValue INTAKE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        /** Neutral mode for pivot motors (coast). */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

        // mechanism characterization
        private static final double ROTOR_TO_MECHANISM_RATIO = 107.6923;
        /** Ratio of sensor units to degrees of pivot rotation. */
        public static final double SENSOR_TO_DEGREE_RATIO = 1 / 360.0;
        /** Position of the pivot axis relative to robot center (inches). */
        public static final Translation2d AXIS_POSITION = new Translation2d(-9.417377, 9.257139);

        // encoder config
        /** CAN ID for the pivot CANcoder. */
        public static final int ENCODER_ID = 53;
        /** Absolute encoder offset for the pivot CANcoder (rotations). */
        public static final double ENCODER_ABSOLUTE_OFFSET = 0.06250135632;
        /** Encoder invert direction for the pivot. */
        public static final SensorDirectionValue ENCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        // up ratchet config
        /** Servo channel for the up ratchet. */
        public static final ServoChannel.ChannelId UP_RATCHET_CHANNEL = ServoChannel.ChannelId.kChannelId1;
        /** Servo pulse width for engaging the up ratchet (microseconds). */
        public static final int UP_RATCHET_ON = 1750;
        /** Servo pulse width for disengaging the up ratchet (microseconds). */
        public static final int UP_RATCHET_OFF = 1450;

        // down ratchet config
        /** Servo channel for the down ratchet. */
        public static final ServoChannel.ChannelId DOWN_RATCHET_CHANNEL = ServoChannel.ChannelId.kChannelId0;
        /** Servo pulse width for engaging the down ratchet (microseconds). */
        public static final int DOWN_RATCHET_ON = 1050;
        /** Servo pulse width for disengaging the down ratchet (microseconds). */
        public static final int DOWN_RATCHET_OFF = 1200;

        // pid & tolerance
        /** Gravity feedforward gain for the pivot (volts). */
        public static final double G = 0.2269;
        /** Velocity feedforward gain (V / (rotor rps)). */
        public static final double V = 7.55 / 2 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps) -> V / (rotor rps)
        /** Acceleration feedforward gain (V / (rotor rps^2)). */
        public static final double A = 0.02 / 2 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps^2) -> V / (rotor rps^2)
        /** Proportional gain for the pivot PID controller. */
        public static final double P = 0.15;
        /** Integral gain for the pivot PID controller. */
        public static final double I = 0;
        /** Derivative gain for the pivot PID controller. */
        public static final double D = 0.01;
        /** Exponential velocity feedforward at reduced voltage allocation. */
        public static final double EXPO_V = V / 0.75; // 75% of the actual max velocity, as it will allocate 1 / 0.75 = 1.33333 times the voltage to 1 rps
        /** Exponential acceleration feedforward at reduced voltage allocation. */
        public static final double EXPO_A = A / 0.0075; // 0.75% of the actual max acceleration
        /** Exponential slow velocity feedforward for precision moves. */
        public static final double EXPO_V_SLOW = V / 0.15; // 15% of the actual max velocity
        /** Safe tolerance for pivot position (degrees). */
        public static final double SAFE_TOLERANCE = 15.0;
        /** Tolerance threshold for considering the pivot at target (degrees). */
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        /** Lower physical limit of pivot travel (degrees). */
        public static final double LOWER_LIMIT = 0;
        /** Upper physical limit of pivot travel (degrees). */
        public static final double UPPER_LIMIT = 105;
        /** Stow position (degrees). */
        public static final double STOW = 75;
        /** Stow position for L2/L3/L4 scoring (degrees). */
        public static final double L2_L3_L4_STOW = 75;
        /** Coral station intake position (degrees). */
        public static final double CORAL_STATION = 50;
        /** Coral station intake position when obstructed (degrees). */
        public static final double CORAL_STATION_OBSTRUCTED = 50;
        /** Ground coral intake position (degrees). */
        public static final double GROUND_CORAL = 3.5;
        /** Ground algae intake position (degrees). */
        public static final double GROUND_ALGAE = 4.5;
        /** L1 coral scoring position (degrees). */
        public static final double L1_CORAL = 31.5;
        /** L2 coral position at branch (degrees). */
        public static final double L2_CORAL_AT_BRANCH = 100.0;
        /** L2 coral position one coral from branch (degrees). */
        public static final double L2_CORAL_ONE_CORAL_FROM_BRANCH = 90.0;
        /** L3 coral position at branch (degrees). */
        public static final double L3_CORAL_AT_BRANCH = 100.6;
        /** L3 coral position one coral from branch (degrees). */
        public static final double L3_CORAL_ONE_CORAL_FROM_BRANCH = 95.6;
        /** L4 coral position at branch (degrees). */
        public static final double L4_CORAL_AT_BRANCH = 90.5;
        /** L4 coral position one coral from branch (degrees). */
        public static final double L4_CORAL_ONE_CORAL_FROM_BRANCH = 80.5;
        /** Low reef algae removal position (degrees). */
        public static final double LOW_REEF_ALGAE = 32.5;
        /** High reef algae removal position (degrees). */
        public static final double HIGH_REEF_ALGAE = 105;
        /** Processor scoring position (degrees). */
        public static final double PROCESSOR = 22.1;
        /** Net scoring position (degrees). */
        public static final double NET = 90;
        /** Prepare climb position (degrees). */
        public static final double PREPARE_CLIMB = 90;
        /** Climb position (degrees). */
        public static final double CLIMB = 10;
    }

    /** Constants for the wrist mechanism. */
    public final static class WristConstants {
        // motor config
        /** CAN ID for the wrist motor. */
        public static final int MOTOR_ID = 61;
        /** Current limit for the wrist motor (amps). */
        public static final double CURRENT_LIMIT = 40;
        /** Motor invert setting for the wrist motor. */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        /** Neutral mode for the wrist motor (coast). */
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

        // mechanism characterization
        private static final double ROTOR_TO_MECHANISM_RATIO = 45.3704;
        /** Ratio of sensor units to degrees of wrist rotation. */
        public static final double SENSOR_TO_DEGREE_RATIO = 1 / 360.0;
        /** Total mass of the wrist (lbs). */
        public static final double MASS_LBS = 4.8121516;
        /** Position of the wrist axis relative to robot center (inches). */
        public static final Translation2d AXIS_POSITION = new Translation2d(-11.767377, 38.007139);
        /** Center of mass offset from the wrist axis at zero position (inches). */
        public static final Translation2d AXIS_TO_ZERO_COM = new Translation2d(3.014233, -4.015809);

        // encoder config
        /** CAN ID for the wrist CANcoder. */
        public static final int ENCODER_ID = 62;
        /** Absolute encoder offset for the wrist CANcoder (rotations). */
        public static final double ENCODER_ABSOLUTE_OFFSET =  0.43460869667;
        /** Encoder invert direction for the wrist. */
        public static final SensorDirectionValue ENCODER_INVERT = SensorDirectionValue.Clockwise_Positive;

        // pid & tolerance
        /** Gravity feedforward function dependent on wrist and pivot angles (volts). */
        public static final BiFunction<Double, Double, Double> G = (wristDeg, pivotDeg) -> 0.15 * Math.sin(Math.toRadians(wristDeg - (90 - pivotDeg)));
        /** Velocity feedforward gain (V / (rotor rps)). */
        public static final double V = 0.7 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps) -> V / (rotor rps)
        /** Acceleration feedforward gain (V / (rotor rps^2)). */
        public static final double A = 0.025 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps^2) -> V / (rotor rps^2)
        /** Proportional gain for the wrist PID controller. */
        public static final double P = 0.2;
        /** Integral gain for the wrist PID controller. */
        public static final double I = 0.0;
        /** Derivative gain for the wrist PID controller. */
        public static final double D = 0.0;
        /** Exponential velocity feedforward at reduced voltage allocation. */
        public static final double EXPO_V = V / 0.8; // 80% of the actual max velocity, as it will allocate 1 / 0.8 = 1.25 times the voltage to 1 rps
        /** Exponential acceleration feedforward at reduced voltage allocation. */
        public static final double EXPO_A = A / 0.05; // 5% of the actual max accel
        /** Safe tolerance for wrist position (degrees). */
        public static final double SAFE_TOLERANCE = 25.0;
        /** Tolerance threshold for considering the wrist at target (degrees). */
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        /** Lower position limit function dependent on elevator and pivot positions (degrees). */
        public static final BiFunction<Double, Double, Double> LOWER_LIMIT = (elevatorPosition, pivotPosition) -> (double) (pivotPosition < 45 ? 125 : elevatorPosition > 1.5 && elevatorPosition < 12 ? 125 : 45);
        /** Upper position limit function dependent on elevator position (degrees). */
        public static final Function<Double, Double> UPPER_LIMIT = (elevatorPosition) -> (double) (elevatorPosition > 5 ? 295 : 160);
        /** Stow position (degrees). */
        public static final double STOW = 125;
        /** Stow position for L2/L3/L4 scoring (degrees). */
        public static final double L2_L3_L4_STOW = 200;
        /** Coral station intake position (degrees). */
        public static final double CORAL_STATION = 125;
        /** Coral station intake position when obstructed (degrees). */
        public static final double CORAL_STATION_OBSTRUCTED = 125;
        /** Ground coral intake position (degrees). */
        public static final double GROUND_CORAL = 150;
        /** Ground algae intake position (degrees). */
        public static final double GROUND_ALGAE = 150;
        /** L1 coral scoring position (degrees). */
        public static final double L1_CORAL = 125;
        /** L2 coral position at branch (degrees). */
        public static final double L2_CORAL_AT_BRANCH = 55;
        /** L2 coral position one coral from branch (degrees). */
        public static final double L2_CORAL_ONE_CORAL_FROM_BRANCH = 45;
        /** L3 coral position at branch (degrees). */
        public static final double L3_CORAL_AT_BRANCH = 55;
        /** L3 coral position one coral from branch (degrees). */
        public static final double L3_CORAL_ONE_CORAL_FROM_BRANCH = 50;
        /** L4 coral position at branch (degrees). */
        public static final double L4_CORAL_AT_BRANCH = 285;
        /** L4 coral position one coral from branch (degrees). */
        public static final double L4_CORAL_ONE_CORAL_FROM_BRANCH = 265;
        /** Low reef algae removal position (degrees). */
        public static final double LOW_REEF_ALGAE = 131.0;
        /** High reef algae removal position (degrees). */
        public static final double HIGH_REEF_ALGAE = 160;
        /** Processor scoring position (degrees). */
        public static final double PROCESSOR = 150;
        /** Net scoring position (degrees). */
        public static final double NET = 175;
        /** Prepare climb position (degrees). */
        public static final double PREPARE_CLIMB = 125;
        /** Climb position (degrees). */
        public static final double CLIMB = 235;
    }

    /** Swerve drivetrain PID and module constants. */
    public static final class SwerveConstants {
        /** Path translation PID proportional gain for PathPlanner. */
        public static final double PATH_TRANSLATION_CONTROLLER_P = 10.0;
        /** Path translation PID derivative gain for PathPlanner. */
        public static final double PATH_TRANSLATION_CONTROLLER_D = 0.01;
        /** Path rotation PID proportional gain for PathPlanner. */
        public static final double PATH_ROTATION_CONTROLLER_P = 7.5;

        /** Angular position PID proportional gain for yaw control. */
        public static final double ANGULAR_POSITION_P = 0.035;
        /** Angular position PID derivative gain for yaw control. */
        public static final double ANGULAR_POSITION_D = 0.0012;

        /** Angular object detection PID proportional gain. */
        public static final double ANGULAR_OBJECT_DETECTION_P = 0.025;
        /** Angular object detection PID derivative gain. */
        public static final double ANGULAR_OBJECT_DETECTION_D = 0.001;

        /** Minimum angle for continuous angular input (-180 degrees). */
        public static final double ANGULAR_MINIMUM_ANGLE = -180.0;
        /** Maximum angle for continuous angular input (180 degrees). */
        public static final double ANGULAR_MAXIMUM_ANGLE = 180.0;

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        /** Steer motor PID and feedforward gains. */
        private static final Slot0Configs STEER_GAINS = new Slot0Configs()
                .withKP(16.756).withKI(0).withKD(0.28988)
                .withKS(0.19849).withKV(2.4115).withKA(0.055522)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        /** Drive motor PID and feedforward gains. */
        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(0.14678).withKI(0).withKD(0)
                .withKS(0.070646).withKV(0.11413).withKA(0.016008);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        /** Closed-loop output type for steer motors (voltage). */
        private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        /** Closed-loop output type for drive motors (voltage). */
        private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        /** Drive motor type (TalonFX integrated). */
        private static final SwerveModuleConstants.DriveMotorArrangement DRIVE_MOTOR_TYPE = SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the steer motor
        /** Steer motor type (TalonFX integrated). */
        private static final SwerveModuleConstants.SteerMotorArrangement STEER_MOTOR_TYPE = SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        /** Steer feedback type using fused CANcoder. */
        private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        /** Stator current limit threshold for slip detection (amps). */
        public static final Current SLIP_CURRENT = Amps.of(65.0);

        /** Audio configuration for the swerve drivetrain (allows music during disable). */
        public static final AudioConfigs AUDIO_CONFIGS = new AudioConfigs().withAllowMusicDurDisable(true);

        // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        /** Initial configuration for drive motors. */
        private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS = new TalonFXConfiguration();
        /** Initial configuration for steer motors with current limits. */
        private static final TalonFXConfiguration STEER_INITIAL_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                // Swerve azimuth does not require much torque output, so we can set a relatively low
                                // stator current limit to help avoid brownouts without impacting performance.
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true)
                );
        /** Initial configuration for CANcoders. */
        private static final CANcoderConfiguration CANCODER_INITIAL_CONFIGS = new CANcoderConfiguration();
        // Configs for the Pigeon 2;
        /** Configuration for the Pigeon 2 IMU. */
        private static final Pigeon2Configuration PIGEON_CONFIGS = new Pigeon2Configuration();

        // Theoretical free speed (m/s) at 12 V applied output;
        /** Theoretical free speed at 12 V (m/s). */
        private static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(5.21);

        // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
        // This may need to be tuned to your individual robot
        /** Coupling ratio between azimuth and drive motor rotations. */
        private static final double COUPLE_RATIO = 3.5714285714285716;

        /** Drive motor gear ratio. */
        private static final double DRIVE_GEAR_RATIO = 6.122448979591837;
        /** Steer motor gear ratio. */
        private static final double STEER_GEAR_RATIO = 21.428571428571427;
        /** Wheel radius. */
        private static final Distance WHEEL_RADIUS = Inches.of(2);

        /** Whether the left side modules are inverted. */
        private static final boolean INVERT_LEFT_SIDE = false;
        /** Whether the right side modules are inverted. */
        private static final boolean INVERT_RIGHT_SIDE = true;

        /** CAN ID for the Pigeon 2 IMU. */
        private static final int PIGEON_ID = 51;

        // Simulation only
        /** Steer motor moment of inertia for simulation. */
        private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
        /** Drive motor moment of inertia for simulation. */
        private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
        // Simulated minimum voltage to overcome friction
        /** Simulated minimum voltage to overcome steer friction. */
        private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
        /** Simulated minimum voltage to overcome drive friction. */
        private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

        /** Swerve drivetrain constants including CAN bus and Pigeon 2 IMU configuration. */
        public static final SwerveDrivetrainConstants SWERVE_DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
                .withCANBusName(CAN_BUS.getName())
                .withPigeon2Id(PIGEON_ID)
                .withPigeon2Configs(PIGEON_CONFIGS);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> CONSTANT_CREATOR
                = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                        .withCouplingGearRatio(COUPLE_RATIO)
                        .withWheelRadius(WHEEL_RADIUS)
                        .withSteerMotorGains(STEER_GAINS)
                        .withDriveMotorGains(DRIVE_GAINS)
                        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT_TYPE)
                        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT_TYPE)
                        .withSlipCurrent(SLIP_CURRENT)
                        .withSpeedAt12Volts(SPEED_AT_12_VOLTS)
                        .withDriveMotorType(DRIVE_MOTOR_TYPE)
                        .withSteerMotorType(STEER_MOTOR_TYPE)
                        .withFeedbackSource(STEER_FEEDBACK_TYPE)
                        .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
                        .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
                        .withEncoderInitialConfigs(CANCODER_INITIAL_CONFIGS)
                        .withSteerInertia(STEER_INERTIA)
                        .withDriveInertia(DRIVE_INERTIA)
                        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);


        // Front Left Module
        /** Constants for the front left swerve module. */
        public static final class FrontLeft {
            private static final int DRIVE_MOTOR_ID = 1;
            private static final int STEER_MOTOR_ID = 11;
            private static final int ENCODER_ID = 21;
            /** Absolute encoder offset for the front left module (rotations). */
            public static final Angle ENCODER_OFFSET = Rotations.of(-0.187255859375);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(13.375);
            private static final Distance Y_POS = Inches.of(10.375);

            /** Pre-built swerve module constants for the front left module. */
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_LEFT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        // Front Right Module
        /** Constants for the front right swerve module. */
        public static final class FrontRight {
            private static final int DRIVE_MOTOR_ID = 2;
            private static final int STEER_MOTOR_ID = 12;
            private static final int ENCODER_ID = 22;
            /** Absolute encoder offset for the front right module (rotations). */
            public static final Angle ENCODER_OFFSET = Rotations.of(-0.121337890625);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(13.375);
            private static final Distance Y_POS = Inches.of(-10.375);

            /** Pre-built swerve module constants for the front right module. */
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_RIGHT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        // Back Left Module
        /** Constants for the back left swerve module. */
        public static final class BackLeft {
            private static final int DRIVE_MOTOR_ID = 3;
            private static final int STEER_MOTOR_ID = 13;
            private static final int ENCODER_ID = 23;
            /** Absolute encoder offset for the back left module (rotations). */
            public static final Angle ENCODER_OFFSET = Rotations.of(-0.146728515625 + 0.55859375);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(-13.375);
            private static final Distance Y_POS = Inches.of(10.375);

            /** Pre-built swerve module constants for the back left module. */
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_LEFT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        // Back Right Module
        /** Constants for the back right swerve module. */
        public static final class BackRight {
            private static final int DRIVE_MOTOR_ID = 4;
            private static final int STEER_MOTOR_ID = 14;
            private static final int ENCODER_ID = 24;
            /** Absolute encoder offset for the back right module (rotations). */
            public static final Angle ENCODER_OFFSET = Rotations.of(0.467529296875);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(-13.375);
            private static final Distance Y_POS = Inches.of(-10.375);


            /** Pre-built swerve module constants for the back right module. */
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_RIGHT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED);
        }
    }
}