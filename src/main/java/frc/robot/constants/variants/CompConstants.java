package frc.robot.constants.variants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.util.ExpUtil;

import java.util.function.BiFunction;
import java.util.function.Function;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Inches;

public final class CompConstants {
    public final static class PhotonConstants {
            public static final String BW_TOP_RIGHT_NAME = "ArducamBW";
            public static final double BW_TOP_RIGHT_FORWARD = 0.409011;
            public static final double BW_TOP_RIGHT_LEFT = -0.266700;
            public static final double BW_TOP_RIGHT_UP = 0.278782;
            public static final double BW_TOP_RIGHT_ROLL = 0.0;
            public static final double BW_TOP_RIGHT_PITCH = -5.0;
            public static final double BW_TOP_RIGHT_YAW = -0.0;

            public static final String BW_TOP_LEFT_NAME = "ArducamBW2";
            public static final double BW_TOP_LEFT_FORWARD = 0.409011;
            public static final double BW_TOP_LEFT_LEFT = 0.266700;
            public static final double BW_TOP_LEFT_UP = 0.278782;
            public static final double BW_TOP_LEFT_ROLL = 0.0;
            public static final double BW_TOP_LEFT_PITCH = -5.0;
            public static final double BW_TOP_LEFT_YAW = 0.0;
    }

    public final static class ElevatorConstants {
        // motor config
        public static final int LOWER_LIMIT_SWITCH_ID = 7;
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        // mechanism characterization
        private static final double ROTOR_TO_PULLEY_RATIO = 11.57;
        private static final double PULLEY_CIRCUMFERENCE = 7.065;
        public static final double ROTOR_TO_INCH_RATIO = ROTOR_TO_PULLEY_RATIO / PULLEY_CIRCUMFERENCE;
        private static final double STAGE_2_LOAD_LBS = 28.44;
        public static final double MASS_LBS = 23.0132625;
        public static final double COM_TO_STAGE_2_RATIO = 0.509767;
        public static final double STAGE_2_LIMIT = 24;
        public static final double COM_TO_STAGE_3_RATIO = 0.3345002;
        public static final Translation2d ZERO_UPRIGHT_COM = new Translation2d(-11.347053, 15.125012);

        // pid & tolerance
        public static final Function<Double, Double> G = (pivotDeg) -> 0.2175 * Math.sin(Math.toRadians(pivotDeg));
        public static final double V = 0.31 / ROTOR_TO_INCH_RATIO; // 1V / (in/s) -> 1V / (rotor rps)
        public static final double A = 0.001 / ROTOR_TO_INCH_RATIO; // 1V / (in/s^2) -> 1V / (rotor rps^2)
        public static final double P = 0.3;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double EXPO_V = V / 0.9; // 90% of the actual max velocity, as it will allocate 1 / 0.9 = 1.1111 times the voltage to 1 rps
        public static final double EXPO_A = A / 0.02; // 2% of the actual max accel
        public static final double SAFE_TOLERANCE = 5.0;
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 46;
        public static final double STOW = 0;
        public static final double CORAL_STATION = 0;
        public static final double GROUND_CORAL = 0;
        public static final double GROUND_ALGAE = 0;
        public static final double L1_CORAL = 6.0;
        public static final double L2_CORAL = 0;
        public static final double L3_CORAL = 18.1;
        public static final double L4_CORAL = 41.5;
        public static final double LOW_REEF_ALGAE = 2.0;
        public static final double HIGH_REEF_ALGAE = 3.0;
        public static final double PROCESSOR = 0;
        public static final double NET = 44;
    }

    public final static class IntakeConstants {
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;
    }

    public final static class PivotConstants {
        // motor config
        public static final InvertedValue PIVOT_INVERT = InvertedValue.Clockwise_Positive;

        // mechanism characterization
        private static final double ROTOR_TO_MECHANISM_RATIO = 107.6923;
        public static final double SENSOR_TO_DEGREE_RATIO = 1 / 360.0;
        public static final Translation2d AXIS_POSITION = new Translation2d(-9.417377, 9.257139);

        // encoder config
        public static final double ENCODER_ABSOLUTE_OFFSET = 0.44409247503;
        public static final SensorDirectionValue ENCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        // ratchet config
        public static final int RATCHET_ON = 1050;
        public static final int RATCHET_OFF = 1200;

        // pid & tolerance
        public static final double G = 0.2269;
        public static final double V = 6.35 / 2 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps) -> V / (rotor rps)
        public static final double A = 0.06 / 2 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps^2) -> V / (rotor rps^2)
        public static final double P = 0.15;
        public static final double I = 0;
        public static final double D = 0.0;
        public static final double EXPO_V = V / 0.4; // 40% of the actual max velocity, as it will allocate 1 / 0.4 = 2.5 times the voltage to 1 rps
        public static final double EXPO_A = A / 0.015; // 1.5% of the actual max acceleration
        public static final double SAFE_TOLERANCE = 15.0;
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        public static final double LOWER_LIMIT = 0;
        public static final double UPPER_LIMIT = 105;
        public static final double STOW = 50;
        public static final double CORAL_STATION = 50;
        public static final double GROUND_CORAL = 3.5;
        public static final double GROUND_ALGAE = 4.5;
        public static final double L1_CORAL = 22.3;
        public static final double L2_CORAL = 100.0;
        public static final double L3_CORAL = 100.6;
        public static final double L4_CORAL = 88.5;
        public static final double LOW_REEF_ALGAE = 32.5;
        public static final double HIGH_REEF_ALGAE = 105;
        public static final double PROCESSOR = 22.1;
        public static final double NET = 90;
    }

    public final static class WristConstants {
        // motor config
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        // mechanism characterization
        private static final double ROTOR_TO_MECHANISM_RATIO = 45.3704;
        public static final double SENSOR_TO_DEGREE_RATIO = 1 / 360.0;
        public static final double MASS_LBS = 4.8121516;
        public static final Translation2d AXIS_POSITION = new Translation2d(-11.767377, 38.007139);
        public static final Translation2d AXIS_TO_ZERO_COM = new Translation2d(3.014233, -4.015809);

        // encoder config
        public static final double ENCODER_ABSOLUTE_OFFSET =  0.02343817819;
        public static final SensorDirectionValue ENCODER_INVERT = SensorDirectionValue.Clockwise_Positive;

        // pid & tolerance
        public static final BiFunction<Double, Double, Double> G = (wristDeg, pivotDeg) -> 0.14 * Math.sin(Math.toRadians(wristDeg - (90 - pivotDeg)));
        public static final double V = 0.7 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps) -> V / (rotor rps)
        public static final double A = 0.015 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps^2) -> V / (rotor rps^2)
        public static final double P = 0.2;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double EXPO_V = V / 0.8; // 80% of the actual max velocity, as it will allocate 1 / 0.8 = 1.25 times the voltage to 1 rps
        public static final double EXPO_A = A / 0.08; // 8% of the actual max accel
        public static final double SAFE_TOLERANCE = 25.0;
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        public static final Function<Double, Double> LOWER_LIMIT = (pivotPosition) -> (double) (pivotPosition < 45 ? 160 : 45);
        public static final Function<Double, Double> UPPER_LIMIT = (elevatorPosition) -> (double) (elevatorPosition > 8 ? 295 : 160);
        public static final double STOW = 125;
        public static final double CORAL_STATION = 125;
        public static final double GROUND_CORAL = 150;
        public static final double GROUND_ALGAE = 150;
        public static final double L1_CORAL = 125;
        public static final double L2_CORAL = 40;
        public static final double L3_CORAL = 55;
        public static final double L4_CORAL = 285;
        public static final double LOW_REEF_ALGAE = 131.0;
        public static final double HIGH_REEF_ALGAE = 160;
        public static final double PROCESSOR = 150;
        public static final double NET = 175;

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
        private static final Current SLIP_CURRENT = Amps.of(65.0);

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
            public static final Angle ENCODER_OFFSET = Rotations.of(0.4365234375);
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
