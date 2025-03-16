package frc.robot.constants.variants;

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
import frc.robot.constants.Constants;
import frc.robot.util.FieldUtil;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.BiFunction;
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

    public static final Distance ROBOT_LENGTH_WITH_BUMPERS = Inches.of(38.5);
    public static final Distance ROBOT_WIDTH_WITH_BUMPERS = Inches.of(32.5);

    public static final Supplier<DriverStation.Alliance> ALLIANCE_SUPPLIER = () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    public static final Function<Supplier<DriverStation.Alliance>, Pose2d> CENTER_OF_LEFT_CORAL_STATION = allianceSupplier ->
            allianceSupplier.get() == DriverStation.Alliance.Red
                    ? FieldUtil.AprilTag.ID_1.pose2d
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero))
                    : FieldUtil.AprilTag.ID_13.pose2d
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero));

    public static final Function<Supplier<DriverStation.Alliance>, Pose2d> CENTER_OF_RIGHT_CORAL_STATION = allianceSupplier ->
            allianceSupplier.get() == DriverStation.Alliance.Red
                    ? FieldUtil.AprilTag.ID_2.pose2d
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero))
                    : FieldUtil.AprilTag.ID_12.pose2d
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), 0, Rotation2d.kZero));

    public static final Function<Supplier<DriverStation.Alliance>, Pose2d> FAR_LEFT_CORAL_STATION = allianceSupplier ->
            allianceSupplier.get() == DriverStation.Alliance.Red
                    ? new Pose2d(Units.inchesToMeters(623.07), Units.inchesToMeters(0), FieldUtil.AprilTag.ID_1.pose2d.getRotation())
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), ROBOT_WIDTH_WITH_BUMPERS.div(2).unaryMinus().in(Meters), Rotation2d.kZero))
                    : new Pose2d(Units.inchesToMeters(67.82), Units.inchesToMeters(316.63), FieldUtil.AprilTag.ID_13.pose2d.getRotation())
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), ROBOT_WIDTH_WITH_BUMPERS.div(2).unaryMinus().in(Meters), Rotation2d.kZero));
    public static final Function<Supplier<DriverStation.Alliance>, Pose2d> FAR_RIGHT_CORAL_STATION = allianceSupplier ->
            allianceSupplier.get() == DriverStation.Alliance.Red
                    ? new Pose2d(Units.inchesToMeters(623.07), Units.inchesToMeters(316.63), FieldUtil.AprilTag.ID_2.pose2d.getRotation())
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), ROBOT_WIDTH_WITH_BUMPERS.div(2).in(Meters), Rotation2d.kZero))
                    : new Pose2d(Units.inchesToMeters(67.82), Units.inchesToMeters(0), FieldUtil.AprilTag.ID_12.pose2d.getRotation())
                            .plus(new Transform2d(ROBOT_LENGTH_WITH_BUMPERS.div(2).in(Meters), ROBOT_WIDTH_WITH_BUMPERS.div(2).in(Meters), Rotation2d.kZero));

    // kSpeedAt12Volts desired top speed
    public static final double MAX_VEL = SwerveConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);
    public static final Function<Double, Double> MAX_CONTROLLED_VEL = elevatorHeight -> MAX_VEL - 0.09 * elevatorHeight;
    // 1.96664381049 rotations per second tuned max angular velocity
    public static final Function<Double, Double> MAX_CONTROLLED_ANGULAR_VEL = elevatorHeight -> RotationsPerSecond.of(0.75).in(RadiansPerSecond) - 0.07 * elevatorHeight;
    public static final Function<Double, Double> MAX_ANGULAR_VEL = elevatorHeight -> elevatorHeight < 16 ? RotationsPerSecond.of(1.96664381049).in(RadiansPerSecond) : MAX_CONTROLLED_ANGULAR_VEL.apply(elevatorHeight);

    public static final double MAX_ACCEL = MetersPerSecondPerSecond.of(10.8).in(MetersPerSecondPerSecond);
    public static final double MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(485.0).in(RadiansPerSecondPerSecond);
    public static final double MAX_CONTROLLED_ACCEL = MetersPerSecondPerSecond.of(5.0).in(MetersPerSecondPerSecond);

    public static final NetworkTableInstance NT_INSTANCE = NetworkTableInstance.getDefault();
    public static final int ONE_MILLION = 1_000_000;
    public static final double DEADBAND = 0.1;

    public static final int SERVO_HUB_ID = 54;
    public static final ServoHub SERVO_HUB = new ServoHub(SERVO_HUB_ID);

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
        public static final String INTAKE_MARKER = "intake";
        public static final String OUTTAKE_MARKER = "outtake";

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                MAX_VEL,
                MAX_CONTROLLED_ACCEL,
                MAX_CONTROLLED_ANGULAR_VEL.apply(0.0),
                MAX_ANGULAR_ACCEL
        );

        public static final double OBJECT_SEARCH_DEGREE_SLANT = 30.0;
        public static final double DEGREE_TOLERANCE_TO_ACCEPT = 2.5;
        public static final double TRANSLATION_TOLERANCE_TO_ACCEPT = 0.03;
        public static final double TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE = 0.5;
        public static final double TRANSLATION_TOLERANCE_TO_TRANSITION = 1.5;
    }

    public static final class VisionConstants {
        public static final Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.03, 0.03, Units.degreesToRadians(0.01));
        public static final Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION =
                dist -> dist < 3.0
                        ? VecBuilder.fill(Math.min(0.03, 0.03 * dist), Math.min(0.03, 0.03 * dist), Units.degreesToRadians(1.0))
                        : VecBuilder.fill(0.05 * dist, 0.05 * dist, Units.degreesToRadians(180.0) * dist);
        public static final Function<Double, Matrix<N3, N1>> VISION_STD_DEV_FUNCTION =
                dist -> dist < 3.0
                        ? VecBuilder.fill(0.075 * dist, 0.075 * dist, Units.degreesToRadians(10.0) * dist)
                        : VecBuilder.fill(0.1 * dist, 0.1 * dist, Units.degreesToRadians(180.0) * dist);

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
            public static final String BW_TOP_RIGHT_NAME = "ArducamBW2";
            public static final double BW_TOP_RIGHT_FORWARD = 0.404;
            public static final double BW_TOP_RIGHT_LEFT = -0.291321;
            public static final double BW_TOP_RIGHT_UP = 0.279631;
            public static final double BW_TOP_RIGHT_ROLL = 0.0;
            public static final double BW_TOP_RIGHT_PITCH = -5.0;
            public static final double BW_TOP_RIGHT_YAW = -30.0;

            public static final String BW_TOP_LEFT_NAME = "ArducamBW";
            public static final double BW_TOP_LEFT_FORWARD = 0.404;
            public static final double BW_TOP_LEFT_LEFT = 0.291321;
            public static final double BW_TOP_LEFT_UP = 0.279631;
            public static final double BW_TOP_LEFT_ROLL = 0.0;
            public static final double BW_TOP_LEFT_PITCH = -5.0;
            public static final double BW_TOP_LEFT_YAW = 30.0;

            public static final String BW_BACK_NAME = "ArducamBW3";
            public static final double BW_BACK_FORWARD = -0.315691;
            public static final double BW_BACK_LEFT = 0.266709;
            public static final double BW_BACK_UP = 0.186127;
            public static final double BW_BACK_ROLL = 0.0;
            public static final double BW_BACK_PITCH = -8.0;
            public static final double BW_BACK_YAW = 180;

            public static final double BW_MAX_TAG_CLEAR_DIST = 5.5;

            public static final double OBJECT_TARGET_PITCH = -15;
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

    public final static class ClimbConstants {
        // motor config
        public static final int ID = 0;
        public static final double CURRENT_LIMIT = 40;
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

        // ratchet
        public static final ServoChannel.ChannelId RATCHET_CHANNEL = ServoChannel.ChannelId.kChannelId1;
        public static final int RATCHET_ON = 1115;
        public static final int RATCHET_OFF = 1250;
        
        // presets
        public static final double IDLE = 0;
        public static final double PREPARE_CLIMB = 225;
        public static final double CLIMB = 45;
    }


    public final static class ElevatorConstants {
        // motor config
        public static final int LEAD_ID = 31;
        public static final int FOLLOWER_ID = 32;
        public static final int LOWER_LIMIT_SWITCH_ID = 0;
        public static final double CURRENT_LIMIT = 40;
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

        // mechanism characterization
        private static final double ROTOR_TO_PULLEY_RATIO = 11.57;
        private static final double PULLEY_CIRCUMFERENCE = 7.065;
        public static final double ROTOR_TO_INCH_RATIO = ROTOR_TO_PULLEY_RATIO / PULLEY_CIRCUMFERENCE;
        private static final double STAGE_2_LOAD_LBS = 28.44;
        public static final double MASS_LBS = 23.0132625;
        public static final double COM_TO_STAGE_2_RATIO = 0.509767;
        public static final double STAGE_3_LIMIT = 22;
        public static final double COM_TO_STAGE_3_RATIO = 0.3345002;
        public static final Translation2d ZERO_UPRIGHT_COM = new Translation2d(-11.347053, 15.125012);

        // pid & tolerance
        public static final Function<Double, Double> G = (pivotDeg) -> 0.2175 * Math.sin(Math.toRadians(pivotDeg));
        public static final double V = 0.31 / ROTOR_TO_INCH_RATIO; // 1V / (in/s) -> 1V / (rotor rps)
        public static final double A = 0.001 / ROTOR_TO_INCH_RATIO; // 1V / (in/s^2) -> 1V / (rotor rps^2)
        public static final double P = 0.25;
        public static final double I = 0.0;
        public static final double D = 0.025;
        public static final double EXPO_V = V / 0.90; // 90% of the actual max velocity, as it will allocate 1 / 0.9 = 1.1111 times the voltage to 1 rps
        public static final double EXPO_A = A / 0.015; // 1.5% of the actual max accel
        public static final double SAFE_TOLERANCE = 5.0;
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 46;
        public static final double STOW = 0;
        public static final double CORAL_STATION = 0;
        public static final double GROUND_CORAL = 0;
        public static final double GROUND_ALGAE = 0;
        public static final double L1_CORAL = 0;
        public static final double L2_CORAL = 0;
        public static final double L3_CORAL = 17.2;
        public static final double L4_CORAL = 41.5;
        public static final double LOW_REEF_ALGAE = 2.0;
        public static final double HIGH_REEF_ALGAE = 3.0;
        public static final double PROCESSOR = 0;
        public static final double NET = 44;
        public static final double CLIMB = 8;
    }

    public final static class IntakeConstants {
        public static final int MOTOR_ID = 41;
        public static final int SENSOR_ID = 42;
        public static final int BEAMBREAK_ID = 4;
        public static final double CURRENT_LIMIT = 40;
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final double DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD = 0.13;
    }

    public final static class PivotConstants {
        // motor config
        public static final int LEAD_ID = 51;
        public static final int FOLLOWER_ID = 52;
        public static final double CURRENT_LIMIT = 40;
        public static final InvertedValue PIVOT_INVERT = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

        // mechanism characterization
        private static final double ROTOR_TO_MECHANISM_RATIO = 107.6923;
        public static final double SENSOR_TO_DEGREE_RATIO = 1 / 360.0;
        public static final Translation2d AXIS_POSITION = new Translation2d(-9.417377, 9.257139);

        // encoder config
        public static final int ENCODER_ID = 53;
        public static final double ENCODER_ABSOLUTE_OFFSET = 0.06250135632;
        public static final SensorDirectionValue ENCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        // ratchet config
        public static final ServoChannel.ChannelId RATCHET_CHANNEL = ServoChannel.ChannelId.kChannelId0;
        public static final int RATCHET_ON = 1050;
        public static final int RATCHET_OFF = 1200;

        // pid & tolerance
        public static final double G = 0.2269;
        public static final double V = 7.55 / 2 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps) -> V / (rotor rps)
        public static final double A = 0.02 / 2 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps^2) -> V / (rotor rps^2)
        public static final double P = 0.15;
        public static final double I = 0;
        public static final double D = 0.01;
        public static final double EXPO_V = V / 0.75; // 75% of the actual max velocity, as it will allocate 1 / 0.75 = 1.33333 times the voltage to 1 rps
        public static final double EXPO_A = A / 0.0075; // 0.75% of the actual max acceleration
        public static final double SAFE_TOLERANCE = 15.0;
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 105;
        public static final double STOW = 50;
        public static final double CORAL_STATION = 50;
        public static final double GROUND_CORAL = 3.5;
        public static final double GROUND_ALGAE = 4.5;
        public static final double L1_CORAL = 31.5;
        public static final double L2_CORAL = 100.0;
        public static final double L3_CORAL = 100.6;
        public static final double L4_CORAL = 90.5;
        public static final double LOW_REEF_ALGAE = 32.5;
        public static final double HIGH_REEF_ALGAE = 105;
        public static final double PROCESSOR = 22.1;
        public static final double NET = 90;
        public static final double CLIMB = 105;
    }

    public final static class WristConstants {
        // motor config
        public static final int MOTOR_ID = 61;
        public static final double CURRENT_LIMIT = 40;
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

        // mechanism characterization
        private static final double ROTOR_TO_MECHANISM_RATIO = 45.3704;
        public static final double SENSOR_TO_DEGREE_RATIO = 1 / 360.0;
        public static final double MASS_LBS = 4.8121516;
        public static final Translation2d AXIS_POSITION = new Translation2d(-11.767377, 38.007139);
        public static final Translation2d AXIS_TO_ZERO_COM = new Translation2d(3.014233, -4.015809);

        // encoder config
        public static final int ENCODER_ID = 62;
        public static final double ENCODER_ABSOLUTE_OFFSET =  0.43460869667;
        public static final SensorDirectionValue ENCODER_INVERT = SensorDirectionValue.Clockwise_Positive;

        // pid & tolerance
        public static final BiFunction<Double, Double, Double> G = (wristDeg, pivotDeg) -> 0.15 * Math.sin(Math.toRadians(wristDeg - (90 - pivotDeg)));
        public static final double V = 0.7 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps) -> V / (rotor rps)
        public static final double A = 0.025 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps^2) -> V / (rotor rps^2)
        public static final double P = 0.2;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double EXPO_V = V / 0.8; // 80% of the actual max velocity, as it will allocate 1 / 0.8 = 1.25 times the voltage to 1 rps
        public static final double EXPO_A = A / 0.05; // 5% of the actual max accel
        public static final double SAFE_TOLERANCE = 25.0;
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        public static final BiFunction<Double, Double, Double> LOWER_LIMIT = (elevatorPosition, pivotPosition) -> (double) (pivotPosition < 45 ? 125 : elevatorPosition > 1.5 && elevatorPosition < 12 ? 125 : 45);
        public static final Function<Double, Double> UPPER_LIMIT = (elevatorPosition) -> (double) (elevatorPosition > 8 ? 295 : 160);
        public static final double STOW = 125;
        public static final double CORAL_STATION = 125;
        public static final double GROUND_CORAL = 150;
        public static final double GROUND_ALGAE = 150;
        public static final double L1_CORAL = 125;
        public static final double L2_CORAL = 55;
        public static final double L3_CORAL = 55;
        public static final double L4_CORAL = 285;
        public static final double LOW_REEF_ALGAE = 131.0;
        public static final double HIGH_REEF_ALGAE = 160;
        public static final double PROCESSOR = 150;
        public static final double NET = 175;
        public static final double CLIMB = 180;
    }

    public static final class SwerveConstants {
        public static final double PATH_TRANSLATION_CONTROLLER_P = 10.0;
        public static final double PATH_ROTATION_CONTROLLER_P = 7.5;

        public static final double ANGULAR_POSITION_P = 0.035;
        public static final double ANGULAR_POSITION_D = 0.0012;

        public static final double ANGULAR_OBJECT_DETECTION_P = 0.025;
        public static final double ANGULAR_OBJECT_DETECTION_D = 0.001;

        public static final double ANGULAR_MINIMUM_ANGLE = -180.0;
        public static final double ANGULAR_MAXIMUM_ANGLE = 180.0;

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs STEER_GAINS = new Slot0Configs()
                .withKP(16.756).withKI(0).withKD(0.28988)
                .withKS(0.19849).withKV(2.4115).withKA(0.055522)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(0.14678).withKI(0).withKD(0)
                .withKS(0.070646).withKV(0.11413).withKA(0.016008);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final SwerveModuleConstants.DriveMotorArrangement DRIVE_MOTOR_TYPE = SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the steer motor
        private static final SwerveModuleConstants.SteerMotorArrangement STEER_MOTOR_TYPE = SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        private static final Current SLIP_CURRENT = Amps.of(65.0);

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
        public static final class FrontLeft {
            private static final int DRIVE_MOTOR_ID = 1;
            private static final int STEER_MOTOR_ID = 11;
            private static final int ENCODER_ID = 21;
            public static final Angle ENCODER_OFFSET = Rotations.of(-0.187255859375);
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
            public static final Angle ENCODER_OFFSET = Rotations.of(-0.121337890625);
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
            public static final Angle ENCODER_OFFSET = Rotations.of(-0.146728515625);
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
            public static final Angle ENCODER_OFFSET = Rotations.of(0.467529296875);
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
