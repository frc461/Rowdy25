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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

/**
 * Robot-specific constant overrides for the test bed robot.
 * <p>
 * Provides camera mount offsets and swerve module configurations specific to
 * the test bed chassis, with a smaller wheelbase than the competition robot.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 */
public final class TestConstants {
    /** CAN bus for the test bed drivetrain. */
    public static final CANBus CAN_BUS = new CANBus("", "./logs/example.hoot");

    /** Vision-related constants for the test bed robot. */
    public static final class VisionConstants {
        /** PhotonVision camera mount constants for the test bed. */
        public static final class PhotonConstants {
            /** Name of the top right black-and-white camera. */
            public static final String BW_TOP_RIGHT_NAME = "ArducamBW2";

            /** Forward mount offset of the top right BW camera (meters). */
            public static final double BW_TOP_RIGHT_FORWARD = Units.inchesToMeters(8.25);

            /** Left mount offset of the top right BW camera (meters). */
            public static final double BW_TOP_RIGHT_LEFT = Units.inchesToMeters(-9.25);

            /** Up mount offset of the top right BW camera (meters). */
            public static final double BW_TOP_RIGHT_UP = 0.0;

            /** Roll mount angle of the top right BW camera (degrees). */
            public static final double BW_TOP_RIGHT_ROLL = 0.0;

            /** Pitch mount angle of the top right BW camera (degrees). */
            public static final double BW_TOP_RIGHT_PITCH = -5.0;

            /** Yaw mount angle of the top right BW camera (degrees). */
            public static final double BW_TOP_RIGHT_YAW = -30.5;

            /** Name of the top left black-and-white camera. */
            public static final String BW_TOP_LEFT_NAME = "ArducamBW";

            /** Forward mount offset of the top left BW camera (meters). */
            public static final double BW_TOP_LEFT_FORWARD = Units.inchesToMeters(8.15);

            /** Left mount offset of the top left BW camera (meters). */
            public static final double BW_TOP_LEFT_LEFT = Units.inchesToMeters(9.25);

            /** Up mount offset of the top left BW camera (meters). */
            public static final double BW_TOP_LEFT_UP = 0.0;

            /** Roll mount angle of the top left BW camera (degrees). */
            public static final double BW_TOP_LEFT_ROLL = 0.0;

            /** Pitch mount angle of the top left BW camera (degrees). */
            public static final double BW_TOP_LEFT_PITCH = -5.0;

            /** Yaw mount angle of the top left BW camera (degrees). */
            public static final double BW_TOP_LEFT_YAW = 27.5;
        }
    }

    /** Swerve drivetrain constants for the test bed robot. */
    public static final class SwerveConstants {
        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        /** Steer motor PID and feedforward gains for the test bed. */
        private static final Slot0Configs STEER_GAINS = new Slot0Configs()
                .withKP(75.0).withKI(0).withKD(0.5)
                .withKS(0.1).withKV(2.66).withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        /** Drive motor PID and feedforward gains for the test bed. */
        private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(0).withKI(0).withKD(0)
                .withKS(0.149).withKV(0.1155).withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        /** Closed-loop output type for steer motors (voltage). */
        private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        /** Closed-loop output type for drive motors (voltage). */
        private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        /** Steer feedback type using fused CANcoder. */
        private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        /** Stator current threshold for slip detection (amps). */
        private static final Current SLIP_CURRENT = Amps.of(120.0);

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
                        .withFeedbackSource(STEER_FEEDBACK_TYPE)
                        .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
                        .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
                        .withEncoderInitialConfigs(CANCODER_INITIAL_CONFIGS)
                        .withSteerInertia(STEER_INERTIA)
                        .withDriveInertia(DRIVE_INERTIA)
                        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);


        /** Constants for the front left swerve module. */
        public static final class FrontLeft {
            /** Drive motor CAN ID for the front left module. */
            private static final int DRIVE_MOTOR_ID = 1;
            /** Steer motor CAN ID for the front left module. */
            private static final int STEER_MOTOR_ID = 11;
            /** Encoder CAN ID for the front left module. */
            private static final int ENCODER_ID = 21;
            /** Absolute encoder offset for the front left module (rotations). */
            private static final Angle ENCODER_OFFSET = Rotation.of(-0.422119140625);
            /** Whether the steer motor is inverted for the front left module. */
            private static final boolean STEER_MOTOR_INVERTED = true;
            /** Whether the CANcoder is inverted for the front left module. */
            private static final boolean CANCODER_INVERTED = false;

            /** X position of the front left module relative to robot center. */
            private static final Distance X_POS = Inches.of(8.25);
            /** Y position of the front left module relative to robot center. */
            private static final Distance Y_POS = Inches.of(8.25);

            /** Pre-built swerve module constants for the front left module. */
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_LEFT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        /** Constants for the front right swerve module. */
        public static final class FrontRight {
            /** Drive motor CAN ID for the front right module. */
            private static final int DRIVE_MOTOR_ID = 2;
            /** Steer motor CAN ID for the front right module. */
            private static final int STEER_MOTOR_ID = 12;
            /** Encoder CAN ID for the front right module. */
            private static final int ENCODER_ID = 22;
            /** Absolute encoder offset for the front right module (rotations). */
            private static final Angle ENCODER_OFFSET = Rotations.of(0.145751953125);
            /** Whether the steer motor is inverted for the front right module. */
            private static final boolean STEER_MOTOR_INVERTED = true;
            /** Whether the CANcoder is inverted for the front right module. */
            private static final boolean CANCODER_INVERTED = false;

            /** X position of the front right module relative to robot center. */
            private static final Distance X_POS = Inches.of(8.25);
            /** Y position of the front right module relative to robot center. */
            private static final Distance Y_POS = Inches.of(-8.25);

            /** Pre-built swerve module constants for the front right module. */
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_RIGHT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        /** Constants for the back left swerve module. */
        public static final class BackLeft {
            /** Drive motor CAN ID for the back left module. */
            private static final int DRIVE_MOTOR_ID = 3;
            /** Steer motor CAN ID for the back left module. */
            private static final int STEER_MOTOR_ID = 13;
            /** Encoder CAN ID for the back left module. */
            private static final int ENCODER_ID = 23;
            /** Absolute encoder offset for the back left module (rotations). */
            private static final Angle ENCODER_OFFSET = Rotations.of(0.39794921875);
            /** Whether the steer motor is inverted for the back left module. */
            private static final boolean STEER_MOTOR_INVERTED = true;
            /** Whether the CANcoder is inverted for the back left module. */
            private static final boolean CANCODER_INVERTED = false;

            /** X position of the back left module relative to robot center. */
            private static final Distance X_POS = Inches.of(-8.25);
            /** Y position of the back left module relative to robot center. */
            private static final Distance Y_POS = Inches.of(8.25);

            /** Pre-built swerve module constants for the back left module. */
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_LEFT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }

        /** Constants for the back right swerve module. */
        public static final class BackRight {
            /** Drive motor CAN ID for the back right module. */
            private static final int DRIVE_MOTOR_ID = 4;
            /** Steer motor CAN ID for the back right module. */
            private static final int STEER_MOTOR_ID = 14;
            /** Encoder CAN ID for the back right module. */
            private static final int ENCODER_ID = 24;
            /** Absolute encoder offset for the back right module (rotations). */
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.22607421875);
            /** Whether the steer motor is inverted for the back right module. */
            private static final boolean STEER_MOTOR_INVERTED = true;
            /** Whether the CANcoder is inverted for the back right module. */
            private static final boolean CANCODER_INVERTED = false;

            /** X position of the back right module relative to robot center. */
            private static final Distance X_POS = Inches.of(-8.25);
            /** Y position of the back right module relative to robot center. */
            private static final Distance Y_POS = Inches.of(-8.25);

            /** Pre-built swerve module constants for the back right module. */
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT = CONSTANT_CREATOR.createModuleConstants(
                    STEER_MOTOR_ID, DRIVE_MOTOR_ID, ENCODER_ID, ENCODER_OFFSET,
                    X_POS, Y_POS, INVERT_RIGHT_SIDE, STEER_MOTOR_INVERTED, CANCODER_INVERTED
            );
        }
    }
}