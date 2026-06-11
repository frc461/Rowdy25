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

/**
 * Robot-specific constant overrides for the ROWDY (competition) robot.
 * <p>
 * Provides calibrated mechanism characterization, PID gains, encoder offsets,
 * and camera mount poses specific to the competition robot hardware.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public final class CompConstants {
    /** PhotonVision camera mount constants for the competition robot. */
    public final static class PhotonConstants {
        /** Name of the top right black-and-white camera. */
        public static final String BW_TOP_RIGHT_NAME = "ArducamBW2";

        /** Forward mount offset of the top right BW camera (meters). */
        public static final double BW_TOP_RIGHT_FORWARD = 0.396311;

        /** Left mount offset of the top right BW camera (meters). */
        public static final double BW_TOP_RIGHT_LEFT = -0.266700 - Units.inchesToMeters(1 / 16.0);

        /** Up mount offset of the top right BW camera (meters). */
        public static final double BW_TOP_RIGHT_UP = 0.254114;

        /** Roll mount angle of the top right BW camera (degrees). */
        public static final double BW_TOP_RIGHT_ROLL = 0.0;

        /** Pitch mount angle of the top right BW camera (degrees). */
        public static final double BW_TOP_RIGHT_PITCH = -5.0;

        /** Yaw mount angle of the top right BW camera (degrees). */
        public static final double BW_TOP_RIGHT_YAW = -0.0;

        /** Name of the top left black-and-white camera. */
        public static final String BW_TOP_LEFT_NAME = "ArducamBW";

        /** Forward mount offset of the top left BW camera (meters). */
        public static final double BW_TOP_LEFT_FORWARD = 0.396311;

        /** Left mount offset of the top left BW camera (meters). */
        public static final double BW_TOP_LEFT_LEFT = 0.266700 + Units.inchesToMeters(1 / 16.0);

        /** Up mount offset of the top left BW camera (meters). */
        public static final double BW_TOP_LEFT_UP = 0.254114;

        /** Roll mount angle of the top left BW camera (degrees). */
        public static final double BW_TOP_LEFT_ROLL = 0.0;

        /** Pitch mount angle of the top left BW camera (degrees). */
        public static final double BW_TOP_LEFT_PITCH = -5.0;

        /** Yaw mount angle of the top left BW camera (degrees). */
        public static final double BW_TOP_LEFT_YAW = 0.0;

        /** Name of the back black-and-white camera. */
        public static final String BW_BACK_NAME = "ArducamBW3";

        /** Forward mount offset of the back BW camera (meters). */
        public static final double BW_BACK_FORWARD = -0.305367;

        /** Left mount offset of the back BW camera (meters). */
        public static final double BW_BACK_LEFT = 0.266457;

        /** Up mount offset of the back BW camera (meters). */
        public static final double BW_BACK_UP = 0.184669;

        /** Roll mount angle of the back BW camera (degrees). */
        public static final double BW_BACK_ROLL = 0.0;

        /** Pitch mount angle of the back BW camera (degrees). */
        public static final double BW_BACK_PITCH = -8.0;

        /** Yaw mount angle of the back BW camera (degrees). */
        public static final double BW_BACK_YAW = 180;
    }

    /** Elevator mechanism constants for the competition robot. */
    public final static class ElevatorConstants {
        // motor config
        /** DIO port for the elevator lower limit switch. */
        public static final int LOWER_LIMIT_SWITCH_DIO_PORT = 7;

        /** Motor invert setting for the elevator motors. */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        // mechanism characterization
        private static final double ROTOR_TO_PULLEY_RATIO = 50.0 / 12.0 * 43.0 / 25.0;
        private static final double PULLEY_CIRCUMFERENCE = 7.06858347058;

        /** Ratio of rotor rotations to inches of elevator travel. */
        public static final double ROTOR_TO_INCH_RATIO = ROTOR_TO_PULLEY_RATIO / PULLEY_CIRCUMFERENCE;
        private static final double STAGE_2_LOAD_LBS = 28.44;

        /** Total mass of the elevator carriage (lbs). */
        public static final double MASS_LBS = 23.0132625 / ((102.2329023 - 54.8422757) / (114.375 - 54.8422757));

        /** Center of mass ratio for stage 2 extension. */
        public static final double COM_TO_STAGE_2_RATIO = 0.509767;

        /** Stage 3 extension limit. */
        public static final double STAGE_3_LIMIT = 22;

        /** Center of mass ratio for stage 3 extension. */
        public static final double COM_TO_STAGE_3_RATIO = 0.3345002;

        /** Center of mass translation at zero upright position. */
        public static final Translation2d ZERO_UPRIGHT_COM = new Translation2d(-11.175605, 14.997186);

        // pid & tolerance
        /** Gravity feedforward function dependent on pivot angle (volts). */
        public static final Function<Double, Double> G = (pivotDeg) -> 0.3513 * Math.sin(Math.toRadians(pivotDeg)); // TODO: REPAIR CONSTANTS

        /** Velocity feedforward gain (V / (rotor rps)). */
        public static final double V = 0.12 / ROTOR_TO_INCH_RATIO; // 1V / (in/s) -> 1V / (rotor rps)

        /** Acceleration feedforward gain (V / (rotor rps^2)). */
        public static final double A = 0.00161498708 / ROTOR_TO_INCH_RATIO; // 1V / (in/s^2) -> 1V / (rotor rps^2)

        /** Proportional gain for the elevator PID controller. */
        public static final double P = 0.3;

        /** Integral gain for the elevator PID controller. */
        public static final double I = 0.0;

        /** Derivative gain for the elevator PID controller. */
        public static final double D = 0.0;

        /** Exponential velocity feedforward at reduced voltage allocation. */
        public static final double EXPO_V = V / 0.95; // 95% of the actual max velocity, as it will allocate 1 / 0.9 = 1.1111 times the voltage to 1 rps

        /** Exponential acceleration feedforward at reduced voltage allocation. */
        public static final double EXPO_A = A / 0.06; // 7.5% of the actual max accel

        /** Safe tolerance for elevator position (inches). */
        public static final double SAFE_TOLERANCE = 15.0;

        /** Tolerance threshold for considering the elevator at target (inches). */
        public static final double AT_TARGET_TOLERANCE = 2.0;

        // presets
        /** Lower physical limit of elevator travel (inches). */
        public static final double LOWER_LIMIT = 0;

        /** Upper physical limit of elevator travel (inches). */
        public static final double UPPER_LIMIT = 46;

        /** Stow position (inches). */
        public static final double STOW = 0;

        /** Stow position for L2/L3/L4 scoring (inches). */
        public static final double L2_L3_L4_STOW = 4.0;

        /** Coral station intake position (inches). */
        public static final double CORAL_STATION = 0;

        /** Coral station intake position when obstructed (inches). */
        public static final double CORAL_STATION_OBSTRUCTED = 3.0;

        /** Ground coral intake position (inches). */
        public static final double GROUND_CORAL = 0;

        /** Ground algae intake position (inches). */
        public static final double GROUND_ALGAE = 0;

        /** L1 coral scoring position (inches). */
        public static final double L1_CORAL = 1.3;

        /** L2 coral position at branch (inches). */
        public static final double L2_CORAL_AT_BRANCH = 0;

        /** L2 coral position one coral from branch (inches). */
        public static final double L2_CORAL_ONE_CORAL_FROM_BRANCH = 5.1;

        /** L3 coral position at branch (inches). */
        public static final double L3_CORAL_AT_BRANCH = 18.5;

        /** L3 coral position one coral from branch (inches). */
        public static final double L3_CORAL_ONE_CORAL_FROM_BRANCH = 15.5;

        /** L4 coral position at branch (inches). */
        public static final double L4_CORAL_AT_BRANCH = 45.5;

        /** L4 coral position one coral from branch (inches). */
        public static final double L4_CORAL_ONE_CORAL_FROM_BRANCH = 40.5;

        /** Low reef algae removal position (inches). */
        public static final double LOW_REEF_ALGAE = 0;

        /** High reef algae removal position (inches). */
        public static final double HIGH_REEF_ALGAE = 19.0;

        /** Processor scoring position (inches). */
        public static final double PROCESSOR = 5.5;

        /** Net scoring position (inches). */
        public static final double NET = 44.5;
    }

    /** Intake mechanism constants for the competition robot. */
    public final static class IntakeConstants {
        /** Motor invert setting for the intake. */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        /** DIO port for the intake beam break sensor. */
        public static final int BEAMBREAK_DIO_PORT = 1;
    }

    /** Pivot mechanism constants for the competition robot. */
    public final static class PivotConstants {
        // motor config
        /** Motor invert setting for the pivot motor. */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        // mechanism characterization
        private static final double ROTOR_TO_MECHANISM_RATIO = 107.6923;

        /** Ratio of sensor units to degrees of pivot rotation. */
        public static final double SENSOR_TO_DEGREE_RATIO = 1 / 360.0;

        /** Position of the pivot axis relative to robot center (inches). */
        public static final Translation2d AXIS_POSITION = new Translation2d(-9.417377, 9.257139);

        // encoder config
        /** Absolute encoder offset for the pivot CANcoder (rotations). */
        public static final double ENCODER_ABSOLUTE_OFFSET = 0.66455615231;

        /** Encoder invert direction for the pivot. */
        public static final SensorDirectionValue ENCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

        // ratchet config
        /** Servo pulse width for engaging the ratchet (microseconds). */
        public static final int RATCHET_ON = 1725;

        /** Servo pulse width for disengaging the ratchet (microseconds). */
        public static final int RATCHET_OFF = 1600;

        // pid & tolerance
        /** Gravity feedforward gain for the pivot (volts). */
        public static final double G = 0.26;

        /** Velocity feedforward gain (V / (rotor rps)). */
        public static final double V = 6.75 / 2 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps) -> V / (rotor rps)

        /** Acceleration feedforward gain (V / (rotor rps^2)). */
        public static final double A = 0.09 / 2 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps^2) -> V / (rotor rps^2)

        /** Proportional gain for the pivot PID controller. */
        public static final double P = 0.15;

        /** Integral gain for the pivot PID controller. */
        public static final double I = 0.0;

        /** Derivative gain for the pivot PID controller. */
        public static final double D = 0.01;

        /** Exponential velocity feedforward at reduced voltage allocation. */
        public static final double EXPO_V = V / 0.45; // 45% of the actual max velocity, as it will allocate 1 / 0.4 = 2.5 times the voltage to 1 rps

        /** Exponential acceleration feedforward at reduced voltage allocation. */
        public static final double EXPO_A = A / 0.025; // 2.5% of the actual max acceleration

        /** Exponential slow velocity feedforward for precision moves. */
        public static final double EXPO_V_SLOW = V / 0.1; // 10% of the actual max velocity

        /** Safe tolerance for pivot position (degrees). */
        public static final double SAFE_TOLERANCE = 20.0;

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
        public static final double CORAL_STATION = 55.2;

        /** Coral station intake position when obstructed (degrees). */
        public static final double CORAL_STATION_OBSTRUCTED = 52;

        /** Ground coral intake position (degrees). */
        public static final double GROUND_CORAL = 3.5;

        /** Ground algae intake position (degrees). */
        public static final double GROUND_ALGAE = 14;

        /** L1 coral scoring position (degrees). */
        public static final double L1_CORAL = 38.0;

        /** L2 coral position at branch (degrees). */
        public static final double L2_CORAL_AT_BRANCH = 95.0;

        /** L2 coral position one coral from branch (degrees). */
        public static final double L2_CORAL_ONE_CORAL_FROM_BRANCH = 89.0;

        /** L3 coral position at branch (degrees). */
        public static final double L3_CORAL_AT_BRANCH = 95.0;

        /** L3 coral position one coral from branch (degrees). */
        public static final double L3_CORAL_ONE_CORAL_FROM_BRANCH = 88.0;

        /** L4 coral position at branch (degrees). */
        public static final double L4_CORAL_AT_BRANCH = 96.0;

        /** L4 coral position one coral from branch (degrees). */
        public static final double L4_CORAL_ONE_CORAL_FROM_BRANCH = 89.0;

        /** Low reef algae removal position (degrees). */
        public static final double LOW_REEF_ALGAE = 57;

        /** High reef algae removal position (degrees). */
        public static final double HIGH_REEF_ALGAE = 90.0;

        /** Processor scoring position (degrees). */
        public static final double PROCESSOR = 11;

        /** Net scoring position (degrees). */
        public static final double NET = 90;
    }

    /** Wrist mechanism constants for the competition robot. */
    public final static class WristConstants {
        // motor config
        /** Motor invert setting for the wrist motor. */
        public static final InvertedValue MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        // mechanism characterization
        private static final double ROTOR_TO_MECHANISM_RATIO = 45.3704;

        /** Ratio of sensor units to degrees of wrist rotation. */
        public static final double SENSOR_TO_DEGREE_RATIO = 1 / 360.0;

        /** Total effective mass of the wrist (lbs). */
        public static final double MASS_LBS = 6.9769122 / ((102.2329023 - 54.8422757) / (114.375 - 54.8422757));

        /** Position of the wrist axis relative to robot center (inches). */
        public static final Translation2d AXIS_POSITION = new Translation2d(-11.767377, 38.007139);

        /** Center of mass offset from the wrist axis at zero position (inches). */
        public static final Translation2d AXIS_TO_ZERO_COM = new Translation2d(-10.440589, 33.398821).minus(AXIS_POSITION);

        // encoder config
        /** Absolute encoder offset for the wrist CANcoder (rotations). */
        public static final double ENCODER_ABSOLUTE_OFFSET = -0.41137491865 + 171.147/360;

        /** Encoder invert direction for the wrist. */
        public static final SensorDirectionValue ENCODER_INVERT = SensorDirectionValue.Clockwise_Positive;

        // pid & tolerance
        /** Gravity feedforward function dependent on wrist and pivot angles (volts). */
        public static final BiFunction<Double, Double, Double> G = (wristDeg, pivotDeg) -> 0.17 * Math.sin(Math.toRadians(wristDeg - (90 - pivotDeg)));

        /** Velocity feedforward gain (V / (rotor rps)). */
        public static final double V = 0.69 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps) -> V / (rotor rps)

        /** Acceleration feedforward gain (V / (rotor rps^2)). */
        public static final double A = 0.02 / ROTOR_TO_MECHANISM_RATIO; // V / (mech rps^2) -> V / (rotor rps^2)

        /** Proportional gain for the wrist PID controller. */
        public static final double P = 0.1;

        /** Integral gain for the wrist PID controller. */
        public static final double I = 0.0;

        /** Derivative gain for the wrist PID controller. */
        public static final double D = 0.0;

        /** Exponential velocity feedforward at reduced voltage allocation. */
        public static final double EXPO_V = V / 0.9; // 90% of the actual max velocity, as it will allocate 1 / 0.8 = 1.25 times the voltage to 1 rps

        /** Exponential acceleration feedforward at reduced voltage allocation. */
        public static final double EXPO_A = A / 0.1; // 10% of the actual max accel

        /** Safe tolerance for wrist position (degrees). */
        public static final double SAFE_TOLERANCE = 50.0;

        /** Tolerance threshold for considering the wrist at target (degrees). */
        public static final double AT_TARGET_TOLERANCE = 2.5;

        // presets
        /** Lower position limit function dependent on elevator and pivot positions (degrees). */
        public static final BiFunction<Double, Double, Double> LOWER_LIMIT = (elevatorPosition, pivotPosition) -> (double) (pivotPosition < 30 ? 45 : elevatorPosition > 1.5 && elevatorPosition < 12 ? 115 : 45);

        /** Upper position limit function dependent on elevator position (degrees). */
        public static final Function<Double, Double> UPPER_LIMIT = (elevatorPosition) -> (double) (elevatorPosition > 3.75 ? 295 : 160);

        /** Stow position (degrees). */
        public static final double STOW = 120;

        /** Stow position for L2/L3/L4 scoring (degrees). */
        public static final double L2_L3_L4_STOW = 200;

        /** Coral station intake position (degrees). */
        public static final double CORAL_STATION = 115;

        /** Coral station intake position when obstructed (degrees). */
        public static final double CORAL_STATION_OBSTRUCTED = 120;

        /** Ground coral intake position (degrees). */
        public static final double GROUND_CORAL = 150;

        /** Ground algae intake position (degrees). */
        public static final double GROUND_ALGAE = 90;

        /** L1 coral scoring position (degrees). */
        public static final double L1_CORAL = 75;

        /** L2 coral position at branch (degrees). */
        public static final double L2_CORAL_AT_BRANCH = 45;

        /** L2 coral position one coral from branch (degrees). */
        public static final double L2_CORAL_ONE_CORAL_FROM_BRANCH = 270;

        /** L3 coral position at branch (degrees). */
        public static final double L3_CORAL_AT_BRANCH = 60;

        /** L3 coral position one coral from branch (degrees). */
        public static final double L3_CORAL_ONE_CORAL_FROM_BRANCH = 270;

        /** L4 coral position at branch (degrees). */
        public static final double L4_CORAL_AT_BRANCH = 75;

        /** L4 coral position one coral from branch (degrees). */
        public static final double L4_CORAL_ONE_CORAL_FROM_BRANCH = 275;

        /** Low reef algae removal position (degrees). */
        public static final double LOW_REEF_ALGAE = 77;

        /** High reef algae removal position (degrees). */
        public static final double HIGH_REEF_ALGAE = 243;

        /** Processor scoring position (degrees). */
        public static final double PROCESSOR = 120;

        /** Net scoring position (degrees). */
        public static final double NET = 165;

    }

    /** Swerve drivetrain constants for the competition robot. */
    public static final class SwerveConstants {
        /** Path translation PID proportional gain for PathPlanner. */
        public static final double PATH_TRANSLATION_CONTROLLER_P = 2.0;

        /** Path rotation PID proportional gain for PathPlanner. */
        public static final double PATH_ROTATION_CONTROLLER_P = 2.0;

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
        /** Steer motor PID and feedforward gains for the competition robot. */
        private static final Slot0Configs STEER_GAINS = new Slot0Configs()
                    .withKP(19.22).withKI(0).withKD(0.49503)
                .withKS(0.15852).withKV(2.4532).withKA(0.089693)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        /** Drive motor PID and feedforward gains for the competition robot. */
        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(0.097116).withKI(0).withKD(0)
                .withKS(0.10746).withKV(0.11507).withKA(0.012509);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        /** Closed-loop output type for steer motors (voltage). */
        private static final SwerveModuleConstants.ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        /** Closed-loop output type for drive motors (voltage). */
        private static final SwerveModuleConstants.ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        /** Drive motor type (TalonFX integrated). */
        private static final SwerveModuleConstants.DriveMotorArrangement DRIVE_MOTOR_TYPE = SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the steer motor
        /** Steer motor type (TalonFX integrated). */
        private static final SwerveModuleConstants.SteerMotorArrangement STEER_MOTOR_TYPE = SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        /** Steer feedback type using fused CANcoder. */
        private static final SwerveModuleConstants.SteerFeedbackType STEER_FEEDBACK_TYPE = SwerveModuleConstants.SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        /** Stator current limit threshold for slip detection (amps). */
        public static final Current SLIP_CURRENT = Amps.of(65.0);

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

        /** Factory for creating swerve module constants with shared configuration. */
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


        /** Constants for the front left swerve module on the competition robot. */
        public static final class FrontLeft {
            private static final int DRIVE_MOTOR_ID = 1;
            private static final int STEER_MOTOR_ID = 11;
            private static final int ENCODER_ID = 21;
            /** Absolute encoder offset for the front left module (rotations). */
            public static final Angle ENCODER_OFFSET = Rotations.of(-1.81684412 + 0.0555555555555556 - 0.56982421875 + 0.51904296875 + .01708984375);
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

        /** Constants for the front right swerve module on the competition robot. */
        public static final class FrontRight {
            private static final int DRIVE_MOTOR_ID = 2;
            private static final int STEER_MOTOR_ID = 12;
            private static final int ENCODER_ID = 22;
            /** Absolute encoder offset for the front right module (rotations). */
            public static final Angle ENCODER_OFFSET = Rotations.of(0.223388671875);
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

        /** Constants for the back left swerve module on the competition robot. */
        public static final class BackLeft {
            private static final int DRIVE_MOTOR_ID = 3;
            private static final int STEER_MOTOR_ID = 13;
            private static final int ENCODER_ID = 23;
            /** Absolute encoder offset for the back left module (rotations). */
            public static final Angle ENCODER_OFFSET = Rotations.of(-0.09521484375);
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

        /** Constants for the back right swerve module on the competition robot. */
        public static final class BackRight {
            private static final int DRIVE_MOTOR_ID = 4;
            private static final int STEER_MOTOR_ID = 14;
            private static final int ENCODER_ID = 24;
            /** Absolute encoder offset for the back right module (rotations). */
            public static final Angle ENCODER_OFFSET = Rotations.of(-0.45947265625);
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