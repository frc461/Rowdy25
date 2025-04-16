package frc.robot.constants.variants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import java.util.function.BiFunction;
import java.util.function.Function;

import static edu.wpi.first.units.Units.*;

public final class CompConstants {
    public final static class PhotonConstants {
            public static final String BW_TOP_RIGHT_NAME = "ArducamBW2";
            public static final double BW_TOP_RIGHT_FORWARD = 0.396311;
            public static final double BW_TOP_RIGHT_LEFT = -0.266700 - Units.inchesToMeters(1 / 16.0);
            public static final double BW_TOP_RIGHT_UP = 0.254114;
            public static final double BW_TOP_RIGHT_ROLL = 0.0;
            public static final double BW_TOP_RIGHT_PITCH = -5.0;
            public static final double BW_TOP_RIGHT_YAW = -0.0;

            public static final String BW_TOP_LEFT_NAME = "ArducamBW";
            public static final double BW_TOP_LEFT_FORWARD = 0.396311;
            public static final double BW_TOP_LEFT_LEFT = 0.266700 + Units.inchesToMeters(1 / 16.0);
            public static final double BW_TOP_LEFT_UP = 0.254114;
            public static final double BW_TOP_LEFT_ROLL = 0.0;
            public static final double BW_TOP_LEFT_PITCH = -5.0;
            public static final double BW_TOP_LEFT_YAW = 0.0;

            public static final String BW_BACK_NAME = "ArducamBW3";
            public static final double BW_BACK_FORWARD = -0.305367;
            public static final double BW_BACK_LEFT = 0.266457;
            public static final double BW_BACK_UP = 0.184669;
            public static final double BW_BACK_ROLL = 0.0;
            public static final double BW_BACK_PITCH = -8.0;
            public static final double BW_BACK_YAW = 180;
    }

    public final static class ElevatorConstants {
        // motor config
        public static final int LOWER_LIMIT_SWITCH_DIO_PORT = 7;
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        // mechanism characterization
        private static final double ROTOR_TO_PULLEY_RATIO = 50.0 / 12.0 * 43.0 / 25.0;
        private static final double PULLEY_CIRCUMFERENCE = 7.06858347058;
        public static final double ROTOR_TO_INCH_RATIO = ROTOR_TO_PULLEY_RATIO / PULLEY_CIRCUMFERENCE;
        private static final double STAGE_2_LOAD_LBS = 28.44;
        public static final double MASS_LBS = 23.0132625 / ((102.2329023 - 54.8422757) / (114.375 - 54.8422757));
        public static final double COM_TO_STAGE_2_RATIO = 0.509767;
        public static final double STAGE_3_LIMIT = 22;
        public static final double COM_TO_STAGE_3_RATIO = 0.3345002;
        public static final Translation2d ZERO_UPRIGHT_COM = new Translation2d(-11.175605, 14.997186);

        // pid & tolerance
        public static final Function<Double, Double> G = (pivotDeg) -> 0.3513 * Math.sin(Math.toRadians(pivotDeg)); // TODO: REPAIR CONSTANTS
        public static final double V = 0.12 / ROTOR_TO_INCH_RATIO; // 1V / (in/s) -> 1V / (rotor rps)
        public static final double A = 0.00161498708 / ROTOR_TO_INCH_RATIO; // 1V / (in/s^2) -> 1V / (rotor rps^2)
        public static final double P = 0.3;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double EXPO_V = V / 0.95; // 95% of the actual max velocity, as it will allocate 1 / 0.9 = 1.1111 times the voltage to 1 rps
        public static final double EXPO_A = A / 0.06; // 7.5% of the actual max accel
        public static final double SAFE_TOLERANCE = 15.0;
        public static final double AT_TARGET_TOLERANCE = 2.0;

        // presets
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 46;
        public static final double STOW = 0;
        public static final double L2_L3_L4_STOW = 4.0;
        public static final double CORAL_STATION = 0;
        public static final double CORAL_STATION_OBSTRUCTED = 3.0;
        public static final double GROUND_CORAL = 0;
        public static final double GROUND_ALGAE = 0;
        public static final double L1_CORAL = 1.3;
        public static final double L2_CORAL_AT_BRANCH = 0;
        public static final double L2_CORAL_ONE_CORAL_FROM_BRANCH = 5.1;
        public static final double L3_CORAL_AT_BRANCH = 18.5;
        public static final double L3_CORAL_ONE_CORAL_FROM_BRANCH = 15.5;
        public static final double L4_CORAL_AT_BRANCH = 45.5;
        public static final double L4_CORAL_ONE_CORAL_FROM_BRANCH = 40.5;
        public static final double LOW_REEF_ALGAE = 0;
        public static final double HIGH_REEF_ALGAE = 19.0;
        public static final double PROCESSOR = 5.5;
        public static final double NET = 44.5;
    }

    public final static class IntakeConstants {
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final int BEAMBREAK_DIO_PORT = 1;
    }

    public final static class PivotConstants {
        // motor config
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        // mechanism characterization
        private static final double ROTOR_TO_MECHANISM_RATIO = 107.6923;
        public static final double SENSOR_TO_DEGREE_RATIO = 1 / 360.0;
        public static final Translation2d AXIS_POSITION = new Translation2d(-9.417377, 9.257139);

        // encoder config
        public static final double ENCODER_ABSOLUTE_OFFSET = 0.66455615231;
        public static final SensorDirectionValue ENCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        // ratchet config
        public static final int RATCHET_ON = 1725;
        public static final int RATCHET_OFF = 1600;

        // pid & tolerance
        public static final double G = 0.26;
        public static final double V = 6.75 / 2 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps) -> V / (rotor rps)
        public static final double A = 0.09 / 2 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps^2) -> V / (rotor rps^2)
        public static final double P = 0.15;
        public static final double I = 0.0;
        public static final double D = 0.01;
        public static final double EXPO_V = V / 0.45; // 45% of the actual max velocity, as it will allocate 1 / 0.4 = 2.5 times the voltage to 1 rps
        public static final double EXPO_A = A / 0.025; // 2.5% of the actual max acceleration
        public static final double EXPO_V_SLOW = V / 0.15; // 15% of the actual max velocity
        public static final double SAFE_TOLERANCE = 20.0;
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 105;
        public static final double STOW = 75;
        public static final double L2_L3_L4_STOW = 75;
        public static final double CORAL_STATION = 57.5;
        public static final double CORAL_STATION_OBSTRUCTED = 53;
        public static final double GROUND_CORAL = 3.5;
        public static final double GROUND_ALGAE = 14;
        public static final double L1_CORAL = 38.0;
        public static final double L2_CORAL_AT_BRANCH = 95.0;
        public static final double L2_CORAL_ONE_CORAL_FROM_BRANCH = 89.0;
        public static final double L3_CORAL_AT_BRANCH = 95.0;
        public static final double L3_CORAL_ONE_CORAL_FROM_BRANCH = 88.0;
        public static final double L4_CORAL_AT_BRANCH = 96.0;
        public static final double L4_CORAL_ONE_CORAL_FROM_BRANCH = 89.0;
        public static final double LOW_REEF_ALGAE = 57;
        public static final double HIGH_REEF_ALGAE = 90.0;
        public static final double PROCESSOR = 11;
        public static final double NET = 90;
    }

    public final static class WristConstants {
        // motor config
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        // mechanism characterization
        private static final double ROTOR_TO_MECHANISM_RATIO = 45.3704;
        public static final double SENSOR_TO_DEGREE_RATIO = 1 / 360.0;
        public static final double MASS_LBS = 6.9769122 / ((102.2329023 - 54.8422757) / (114.375 - 54.8422757));
        public static final Translation2d AXIS_POSITION = new Translation2d(-11.767377, 38.007139);
        public static final Translation2d AXIS_TO_ZERO_COM = new Translation2d(-10.440589, 33.398821).minus(AXIS_POSITION);

        // encoder config
        public static final double ENCODER_ABSOLUTE_OFFSET = -0.41137491865;
        public static final SensorDirectionValue ENCODER_INVERT = SensorDirectionValue.Clockwise_Positive;

        // pid & tolerance
        public static final BiFunction<Double, Double, Double> G = (wristDeg, pivotDeg) -> 0.17 * Math.sin(Math.toRadians(wristDeg - (90 - pivotDeg)));
        public static final double V = 0.69 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps) -> V / (rotor rps)
        public static final double A = 0.02 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps^2) -> V / (rotor rps^2)
        public static final double P = 0.1;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double EXPO_V = V / 0.9; // 90% of the actual max velocity, as it will allocate 1 / 0.8 = 1.25 times the voltage to 1 rps
        public static final double EXPO_A = A / 0.1; // 10% of the actual max accel
        public static final double SAFE_TOLERANCE = 50.0;
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        public static final BiFunction<Double, Double, Double> LOWER_LIMIT = (elevatorPosition, pivotPosition) -> (double) (pivotPosition < 30 ? 45 : elevatorPosition > 1.5 && elevatorPosition < 12 ? 115 : 45);
        public static final Function<Double, Double> UPPER_LIMIT = (elevatorPosition) -> (double) (elevatorPosition > 3.75 ? 295 : 160);
        public static final double STOW = 120;
        public static final double L2_L3_L4_STOW = 200;
        public static final double CORAL_STATION = 115;
        public static final double CORAL_STATION_OBSTRUCTED = 120;
        public static final double GROUND_CORAL = 150;
        public static final double GROUND_ALGAE = 90;
        public static final double L1_CORAL = 75;
        public static final double L2_CORAL_AT_BRANCH = 45;
        public static final double L2_CORAL_ONE_CORAL_FROM_BRANCH = 270;
        public static final double L3_CORAL_AT_BRANCH = 60;
        public static final double L3_CORAL_ONE_CORAL_FROM_BRANCH = 270;
        public static final double L4_CORAL_AT_BRANCH = 75;
        public static final double L4_CORAL_ONE_CORAL_FROM_BRANCH = 275;
        public static final double LOW_REEF_ALGAE = 85;
        public static final double HIGH_REEF_ALGAE = 245;
        public static final double PROCESSOR = 120;
        public static final double NET = 165;

    }

    public static final class SwerveConstants {
        public static final double PATH_TRANSLATION_CONTROLLER_P = 2.0;
        public static final double PATH_ROTATION_CONTROLLER_P = 2.0;

        public static final double ANGULAR_POSITION_P = 0.035;
        public static final double ANGULAR_POSITION_D = 0.0012;

        public static final double ANGULAR_OBJECT_DETECTION_P = 0.025;
        public static final double ANGULAR_OBJECT_DETECTION_D = 0.001;

        public static final double ANGULAR_MINIMUM_ANGLE = -180.0;
        public static final double ANGULAR_MAXIMUM_ANGLE = 180.0;

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs STEER_GAINS = new Slot0Configs()
                    .withKP(19.22).withKI(0).withKD(0.49503)
                .withKS(0.15852).withKV(2.4532).withKA(0.089693)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(0.097116).withKI(0).withKD(0)
                .withKS(0.10746).withKV(0.11507).withKA(0.012509);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final SwerveModuleConstants.ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final SwerveModuleConstants.ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final SwerveModuleConstants.DriveMotorArrangement DRIVE_MOTOR_TYPE = SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the steer motor
        private static final SwerveModuleConstants.SteerMotorArrangement STEER_MOTOR_TYPE = SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        private static final SwerveModuleConstants.SteerFeedbackType STEER_FEEDBACK_TYPE = SwerveModuleConstants.SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        public static final Current SLIP_CURRENT = Amps.of(65.0);

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

        // Simulation only
        private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
        // Simulated minimum voltage to overcome friction
        private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
        private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

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
            public static final Angle ENCODER_OFFSET = Rotations.of(-1.81684412 + 0.0555555555555556 - 0.56982421875 + 0.51904296875 + .01708984375);
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
            public static final Angle ENCODER_OFFSET = Rotations.of(0.223388671875);
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
            public static final Angle ENCODER_OFFSET = Rotations.of(-0.09521484375);
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
            public static final Angle ENCODER_OFFSET = Rotations.of(-0.45947265625);
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
