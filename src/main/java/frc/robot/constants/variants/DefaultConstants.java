package frc.robot.constants.variants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.ExpUtil;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.Function;
import java.util.function.Supplier;

public final class DefaultConstants {

    // CAN bus that the devices are located on;
    // If there is more than one CAN bus, create a CANBus constant for each one
    public static final CANBus CAN_BUS = new CANBus("", "./logs/example.hoot");

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    public static final Rotation2d BLUE_DEFAULT_ROTATION = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    public static final Rotation2d RED_DEFAULT_ROTATION = Rotation2d.fromDegrees(180);

    public static final Supplier<DriverStation.Alliance> ALLIANCE_SUPPLIER = () -> DriverStation.getAlliance().orElse(null);

    // kSpeedAt12Volts desired top speed
    public static final double MAX_VEL = SwerveConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);
    // 1.96664381049 rotations per second tuned max angular velocity
    public static final double MAX_ANGULAR_VEL = RotationsPerSecond.of(1.96664381049).in(RadiansPerSecond);
    public static final double MAX_CONTROLLED_ANGULAR_VEL = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public static final double MAX_ACCEL = MetersPerSecondPerSecond.of(10.8).in(MetersPerSecondPerSecond);
    public static final double MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(485.0).in(RadiansPerSecondPerSecond);
    public static final double MAX_CONTROLLED_ACCEL = MetersPerSecondPerSecond.of(5.0).in(MetersPerSecondPerSecond);

    public static final NetworkTableInstance NT_INSTANCE = NetworkTableInstance.getDefault();
    public static final int ONE_MILLION = 1_000_000;
    public static final double DEADBAND = 0.1;

    public static final class AutoConstants {
        public static final RobotConfig ROBOT_CONFIG;

        static {
            try {
                ROBOT_CONFIG = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
        }

        public static final String ALGAE_CHECK_MARKER = "checkAlgae";

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                MAX_VEL,
                MAX_CONTROLLED_ACCEL,
                MAX_CONTROLLED_ANGULAR_VEL,
                MAX_ANGULAR_ACCEL
        );

        public static final double OBJECT_SEARCH_DEGREE_SLANT = 30.0;
        public static final double DEGREE_TOLERANCE_TO_DRIVE_INTO = 2.5;
        public static final double TRANSLATION_TOLERANCE_TO_ACCEPT = 0.2;
        public static final double DISTANCE_TOLERANCE_TO_DRIVE_INTO = 1.25;
    }

    public static final class VisionConstants {
        public static final Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.01));
        public static final Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION =
                dist -> dist < 2.0
                        ? VecBuilder.fill(Math.min(0.1, 0.1 * dist), Math.min(0.1, 0.1 * dist), Units.degreesToRadians(1.0))
                        : VecBuilder.fill(0.15 * dist, 0.15 * dist, Units.degreesToRadians(180.0) * dist);
        public static final Function<Double, Matrix<N3, N1>> VISION_STD_DEV_FUNCTION =
                dist -> dist < 2.0
                        ? VecBuilder.fill(0.15 * dist, 0.15 * dist, Units.degreesToRadians(10.0) * dist)
                        : VecBuilder.fill(0.5 * dist, 0.5 * dist, Units.degreesToRadians(180.0 * dist)); // TODO SHOP: TEST THIS LESS STRICT STD DEV FUNCTION

        public static final class LimelightConstants {
            public static final String LIMELIGHT_NT_NAME = "limelight";

            public static final double LL_FORWARD = 0.0;
            public static final double LL_RIGHT = 0.0;
            public static final double LL_UP = 0.0;
            public static final double LL_ROLL = 0.0;
            public static final double LL_PITCH = 0.0;
            public static final double LL_YAW = 0.0;

            public static final double LL_MAX_TAG_CLEAR_DIST = 4.0;
        }

        public static final class PhotonConstants {
            // TODO SHOP: TEST CAMERAS TO CENTER OF ROBOT OFFSETS
            public static final String BW_TOP_RIGHT_NAME = "ArducamBW";
            public static final double BW_TOP_RIGHT_FORWARD = 0.0;
            public static final double BW_TOP_RIGHT_LEFT = 0.0;
            public static final double BW_TOP_RIGHT_UP = 0.0;
            public static final double BW_TOP_RIGHT_ROLL = 0.0;
            public static final double BW_TOP_RIGHT_PITCH = 0.0;
            public static final double BW_TOP_RIGHT_YAW = 0.0;

            public static final String BW_TOP_LEFT_NAME = "ArducamBW2";
            public static final double BW_TOP_LEFT_FORWARD = 0.0;
            public static final double BW_TOP_LEFT_LEFT = 0.0;
            public static final double BW_TOP_LEFT_UP = 0.0;
            public static final double BW_TOP_LEFT_ROLL = 0.0;
            public static final double BW_TOP_LEFT_PITCH = 0.0;
            public static final double BW_TOP_LEFT_YAW = 0.0;

            public static final String BW_BACK_NAME = "ArducamBW3";
            public static final double BW_BACK_FORWARD = 0.0;
            public static final double BW_BACK_LEFT = 0.0;
            public static final double BW_BACK_UP = 0.0;
            public static final double BW_BACK_ROLL = 0.0;
            public static final double BW_BACK_PITCH = 0.0;
            public static final double BW_BACK_YAW = 0.0;

            public static final double BW_MAX_TAG_CLEAR_DIST = 3;

            public static final double OBJECT_GOAL_PITCH = -15;
        }

        public static final class QuestNavConstants {
            public static final String QUESTNAV_NT_NAME = "questnav";

            // TODO WAIT (NEXT QUEST NAV UPDATE): SET QUEST TO CENTER OF ROBOT OFFSETS
            public static final double QUEST_FORWARD = Units.inchesToMeters(-2.5);
            public static final double QUEST_LEFT = Units.inchesToMeters(5.25);
            public static final double QUEST_UP = 0.0;
            public static final double QUEST_ROLL = 0.0;
            public static final double QUEST_PITCH = 0.0;
            public static final double QUEST_YAW = 0.0;

            // The error threshold to cross when QuestNav's correctional offset will be re-corrected by the error amount.
            public static final double TRANSLATION_ERROR_TOLERANCE = 0.1;
            public static final double ROTATION_ERROR_TOLERANCE = 3.0;

            public static final double MIN_TAG_DIST_TO_BE_FAR = 5.0;
        }
    }

    // TODO SHOP: UPDATE VALUES FOR 2025 + TUNE
    public final static class ElevatorConstants {
        // basic configs
        public static final int LEAD_ID = 31;
        public static final int FOLLOWER_ID = 32;
        public static final int LOWER_LIMIT_SWITCH_ID = 2;
        public static final int CURRENT_LIMIT = 60;
        public static final InvertedValue ELEVATOR_INVERT = InvertedValue.Clockwise_Positive; // TODO SHOP: CHECK ON REAL ROBOT

        // pid
        public static final double ELEVATOR_S = 0.0;
        public static final double ELEVATOR_V = 0.0;
        public static final double ELEVATOR_A = 0.0;
        public static final double ELEVATOR_P = 0.0;
        public static final double ELEVATOR_I = 0.0;
        public static final double ELEVATOR_D = 0.0;

        // presets
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 37;
        public static final double GROUND_CORAL = 0;
        public static final double GROUND_ALGAE = 0;
        public static final double L1_CORAL = 0;
        public static final double L2_CORAL = 0;
        public static final double L3_CORAL = 0;
        public static final double L4_CORAL = 0;
        public static final double LOW_REEF_ALGAE = 0;
        public static final double HIGH_REEF_ALGAE = 0;
        public static final double PROCESSOR = 0;
        public static final double NET = 0;
    }

    public final static class IntakeConstants {
        // basic configs
        public static final int MOTOR_ID = 41;
        public static final int CORAL_BEAM_ID = 3;
        public static final int ALGAE_BEAM_ID = 4;
        public static final int CURRENT_LIMIT = 40;
        public static final InvertedValue INVERT = InvertedValue.Clockwise_Positive; // TODO SHOP: CHECK ON REAL ROBOT
    }

    public final static class PivotConstants {
        // basic configs
        public static final int LEAD_ID = 51;
        public static final int FOLLOWER_ID = 52;
        public static final int ENCODER_ID = 53;
        public static final int RATCHET_ID = 1;
        public static final int LOWER_LIMIT_SWITCH_ID = 0;
        public static final int UPPER_LIMIT_SWITCH_ID = 0;
        public static final int CURRENT_LIMIT = 60;
        public static final InvertedValue PIVOT_INVERT = InvertedValue.Clockwise_Positive; // TODO SHOP: CHECK ON REAL ROBOT

        // pid
        public static final double PIVOT_S = 0.0;
        public static final double PIVOT_V = 0.0;
        public static final double PIVOT_A = 0.0;
        public static final double PIVOT_P = 0;
        public static final double PIVOT_I = 0;
        public static final double PIVOT_D = 0;

        // presets
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 0;
        public static final double CORAL_STATION = 0;
        public static final double GROUND_ALGAE = 0;
        public static final double GROUND_CORAL = 0;
        public static final double SCORE_CORAL = 0;
        public static final double SCORE_ALGAE = 0;
        public static final double STOW_POSITION = 0;
        public static final double TOLERANCE = 0;

        public static final double RATCHET_ON = 0;
        public static final double RATCHET_OFF = 0;
    }

    public final static class WristConstants {
        // basic configs
        public static final int MOTOR_ID = 61;
        public static final int ENCODER_ID = 62;
        public static final int LOWER_LIMIT_SWITCH_ID = 6;
        public static final int UPPER_LIMIT_SWITCH_ID = 0;
        public static final int CURRENT_LIMIT = 35;
        public static final int SENSOR_TO_DEGREE_RATIO = 1;
        public static final InvertedValue WRIST_INVERT = InvertedValue.Clockwise_Positive; // TODO SHOP: CHECK ON REAL ROBOT

        // pid
        public static final double WRIST_S = 0.0;
        public static final double WRIST_V = 0.0;
        public static final double WRIST_A = 0.0;
        public static final double WRIST_P = 0.0;
        public static final double WRIST_I = 0.0;
        public static final double WRIST_D = 0.0;

        // presets
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 20;
        public static final double GROUND_CORAL = 0;
        public static final double GROUND_ALGAE = 0;
        public static final double L1_CORAL = 0;
        public static final double L2_L3_CORAL = 0;
        public static final double L4_CORAL = 0;
        public static final double REEF_ALGAE = 0;
        public static final double PROCESSOR = 0;
        public static final double NET = 0;

    }

    public static final class SwerveConstants {
        public static final double PATH_TRANSLATION_CONTROLLER_P = 10.0;
        public static final double PATH_ROTATION_CONTROLLER_P = 7.5;

        public static final double TRANSLATION_ALIGNMENT_CONTROLLER_P = 1.0;
        public static final double TRANSLATION_ALIGNMENT_CONTROLLER_D = 0.002;

        public static final Function<Double, Double> PATH_MANUAL_TRANSLATION_CONTROLLER = x -> ExpUtil.output(x, 4.0, 0.8, 6);

        public static final double ANGULAR_POSITION_P = 0.035;
        public static final double ANGULAR_POSITION_D = 0.0012;

        public static final double ANGULAR_OBJECT_DETECTION_P = 0.025;
        public static final double ANGULAR_OBJECT_DETECTION_D = 0.001;

        public static final double ANGULAR_MINIMUM_ANGLE = -180.0;
        public static final double ANGULAR_MAXIMUM_ANGLE = 180.0;

        // TODO SHOP: TUNE FOR 2025 ROBOT

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs STEER_GAINS = new Slot0Configs()
                .withKP(75.0).withKI(0).withKD(0.5)
                .withKS(0.1).withKV(2.66).withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(0).withKI(0).withKD(0)
                .withKS(0.149).withKV(0.1155).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // TODO SHOP: TUNE FOR 2025 ROBOT
        private static final Current SLIP_CURRENT = Amps.of(120.0);

        public static final AudioConfigs AUDIO_CONFIGS = new AudioConfigs().withAllowMusicDurDisable(true);

        // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS = new TalonFXConfiguration();
        private static final TalonFXConfiguration STEER_INITIAL_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                // Swerve azimuth does not require much torque output, so we can set a relatively low
                                // stator current limit to help avoid brownouts without impacting performance.
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true)
                );
        private static final CANcoderConfiguration CANCODER_INITIAL_CONFIGS = new CANcoderConfiguration();
        // Configs for the Pigeon 2;
        private static final Pigeon2Configuration PIGEON_CONFIGS = new Pigeon2Configuration();

        // Theoretical free speed (m/s) at 12 V applied output;
        private static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(5.21);

        // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double COUPLE_RATIO = 3.5714285714285716;

        private static final double DRIVE_GEAR_RATIO = 6.122448979591837;
        private static final double STEER_GEAR_RATIO = 21.428571428571427;
        private static final Distance WHEEL_RADIUS = Inches.of(2);

        private static final boolean INVERT_LEFT_SIDE = false;
        private static final boolean INVERT_RIGHT_SIDE = true;

        private static final int PIGEON_ID = 51;

        // Simulation only
        private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
        // Simulated minimum voltage to overcome friction
        private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
        private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

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
                        .withFeedbackSource(STEER_FEEDBACK_TYPE)
                        .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
                        .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
                        .withEncoderInitialConfigs(CANCODER_INITIAL_CONFIGS)
                        .withSteerInertia(STEER_INERTIA)
                        .withDriveInertia(DRIVE_INERTIA)
                        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);


        // Front Left Module
        public static final class FrontLeft {
            private static final int DRIVE_MOTOR_ID = 1;
            private static final int STEER_MOTOR_ID = 11;
            private static final int ENCODER_ID = 21;
            private static final Angle ENCODER_OFFSET = Rotations.of(0.466552734375);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(13.375);
            private static final Distance Y_POS = Inches.of(10.375);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_LEFT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        // Front Right Module
        public static final class FrontRight {
            private static final int DRIVE_MOTOR_ID = 2;
            private static final int STEER_MOTOR_ID = 12;
            private static final int ENCODER_ID = 22;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.091796875);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(13.375);
            private static final Distance Y_POS = Inches.of(-10.375);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_RIGHT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        // Back Left Module
        public static final class BackLeft {
            private static final int DRIVE_MOTOR_ID = 3;
            private static final int STEER_MOTOR_ID = 13;
            private static final int ENCODER_ID = 23;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.120361328125);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(-13.375);
            private static final Distance Y_POS = Inches.of(10.375);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_LEFT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        // Back Right Module
        public static final class BackRight {
            private static final int DRIVE_MOTOR_ID = 4;
            private static final int STEER_MOTOR_ID = 14;
            private static final int ENCODER_ID = 24;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.19921875);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(-13.375);
            private static final Distance Y_POS = Inches.of(-10.375);


            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_RIGHT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED);
        }
    }
}
