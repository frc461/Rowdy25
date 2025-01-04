package frc.robot.constants.variants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    public static final double MAX_REAL_ANGULAR_VEL = RotationsPerSecond.of(1.96664381049).in(RadiansPerSecond);
    public static final double MAX_CONTROLLED_ANGULAR_VEL = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    public static final double MAX_ACCEL = MetersPerSecondPerSecond.of(10.8).in(MetersPerSecondPerSecond);
    public static final double MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(485.0).in(RadiansPerSecondPerSecond);

    public static final NetworkTableInstance NT_INSTANCE = NetworkTableInstance.getDefault();
    public static final int ONE_MILLION = 1_000_000;

    public static final class AutoConstants {
        public static final RobotConfig ROBOT_CONFIG;

        static {
            try {
                ROBOT_CONFIG = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
        }

        // TODO: REMOVE ALL MENTIONS OF "NOTE"

        public static final String NOTE_CHECK_MARKER = "checkNote";

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                MAX_VEL,
                MAX_ACCEL,
                MAX_CONTROLLED_ANGULAR_VEL,
                MAX_ANGULAR_ACCEL
        );

        public static final double NOTE_SEARCH_DEGREE_SLANT = 30.0;
        public static final double DEGREE_TOLERANCE_TO_ACCEPT = 2.5;
        public static final double TRANSLATION_TOLERANCE_TO_ACCEPT = 0.5;
    }

    public static final class VisionConstants {
        public static final Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.01));
        public static final Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION =
                dist -> dist < 2.0
                        ? VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1.0))
                        : VecBuilder.fill(0.15 * dist, 0.15 * dist, Units.degreesToRadians(180.0) * dist);
        public static final Function<Double, Matrix<N3, N1>> VISION_STD_DEV_FUNCTION =
                dist -> VecBuilder.fill(0.5 * dist, 0.5 * dist, Units.degreesToRadians(180.0) * dist);

        public static final class LimelightConstants {
            public static final String LIMELIGHT_NT_NAME = "limelight";

            // TODO SET LL CAMERA TO CENTER OF ROBOT OFFSETS
            public static final double LL_FORWARD = 0.0;
            public static final double LL_RIGHT = 0.0;
            public static final double LL_UP = Units.inchesToMeters(22.5);
            public static final double LL_ROLL = 0.0;
            public static final double LL_PITCH = 25.5;
            public static final double LL_YAW = 0.0;

            public static final double LL_MAX_TAG_CLEAR_DIST = 4.0;
        }

        public static final class PhotonConstants {
            // TODO SET ARDUCAM BW CAMERA TO CENTER OF ROBOT OFFSETS
            public static final double BW_FORWARD = 0.0;
            public static final double BW_LEFT = Units.inchesToMeters(-4.0);
            public static final double BW_UP = 0.0;
            public static final double BW_ROLL = 0.0;
            public static final double BW_PITCH = 25.5;
            public static final double BW_YAW = 0.0;

            public static final double BW_MAX_TAG_CLEAR_DIST = 6.0;

            public static final double OBJECT_GOAL_PITCH = -15;
            public static final double OBJECT_DEGREE_TOLERANCE_TO_ACCEPT = 2.5;
        }

        public static final class QuestNavConstants {
            public static final String QUESTNAV_NT_NAME = "questnav";

            // TODO SET QUEST TO CENTER OF ROBOT OFFSETS
            public static final double QUEST_FORWARD = Units.inchesToMeters(-2.5);
            public static final double QUEST_LEFT = Units.inchesToMeters(5.25);
            public static final double QUEST_UP = 0.0;
            public static final double QUEST_ROLL = 0.0;
            public static final double QUEST_PITCH = 0.0;
            public static final double QUEST_YAW = 0.0;

            // The thresholds through which the QuestNav's correctional offset will be recorrected by the error amount.
            public static final double TRANSLATION_ERROR_TOLERANCE = 0.1;
            public static final double ROTATION_ERROR_TOLERANCE = 3.0;

            public static final double MIN_TAG_DIST_TO_BE_FAR = 5.0;
        }
    }

    public static final class SwerveConstants {
        public static final double PATH_TRANSLATION_CONTROLLER_P = 10.0;
        public static final double PATH_ROTATION_CONTROLLER_P = 7.5;

        public static Function<Double, Double> PATH_MANUAL_TRANSLATION_CONTROLLER = x -> ExpUtil.output(x, 0.8, 6);

        public static final double ANGULAR_POSITION_P = 0.035;
        public static final double ANGULAR_POSITION_D = 0.0012;

        public static final double ANGULAR_OBJECT_DETECTION_P = 0.025;
        public static final double ANGULAR_OBJECT_DETECTION_D = 0.001;

        public static final double ANGULAR_MINIMUM_ANGLE = -180.0;
        public static final double ANGULAR_MAXIMUM_ANGLE = 180.0;

        // TODO TUNE FOR 2025 ROBOT

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
        // TODO TUNE TO 2025 ROBOT
        private static final Current SLIP_CURRENT = Amps.of(120.0);

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
        // TODO TUNE TO 2025 ROBOT
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

        private static final SwerveModuleConstantsFactory CONSTANT_CREATOR = new SwerveModuleConstantsFactory()
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
                .withCANcoderInitialConfigs(CANCODER_INITIAL_CONFIGS)
                .withSteerInertia(STEER_INERTIA)
                .withDriveInertia(DRIVE_INERTIA)
                .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);


        // Front Left Module
        public static final class FrontLeft {
            private static final int DRIVE_MOTOR_ID = 1;
            private static final int STEER_MOTOR_ID = 11;
            private static final int ENCODER_ID = 21;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.422119140625);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(8.25);
            private static final Distance Y_POS = Inches.of(8.25);

            public static final SwerveModuleConstants FRONT_LEFT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_LEFT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        // Front Right Module
        public static final class FrontRight {
            private static final int DRIVE_MOTOR_ID = 2;
            private static final int STEER_MOTOR_ID = 12;
            private static final int ENCODER_ID = 22;
            private static final Angle ENCODER_OFFSET = Rotations.of(0.145751953125);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(8.25);
            private static final Distance Y_POS = Inches.of(-8.25);

            public static final SwerveModuleConstants FRONT_RIGHT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_RIGHT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        // Back Left Module
        public static final class BackLeft {
            private static final int DRIVE_MOTOR_ID = 3;
            private static final int STEER_MOTOR_ID = 13;
            private static final int ENCODER_ID = 23;
            private static final Angle ENCODER_OFFSET = Rotations.of(0.39794921875);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(-8.25);
            private static final Distance Y_POS = Inches.of(8.25);

            public static final SwerveModuleConstants BACK_LEFT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_LEFT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        // Back Right Module
        public static final class BackRight {
            private static final int DRIVE_MOTOR_ID = 4;
            private static final int STEER_MOTOR_ID = 14;
            private static final int ENCODER_ID = 24;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.22607421875);
            private static final boolean STEER_MOTOR_INVERTED = true;
            private static final boolean CANCODER_INVERTED = false;

            private static final Distance X_POS = Inches.of(-8.25);
            private static final Distance Y_POS = Inches.of(-8.25);

            public static final SwerveModuleConstants BACK_RIGHT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_RIGHT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED);
        }
    }
}
