package io.github.frc461.rowdy25.subsystems.pivot;

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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.constants.RobotPoses;
import io.github.frc461.rowdy25.util.EquationUtil;
import io.github.frc461.rowdy25.util.GravityGainsCalculator;
import io.github.frc461.rowdy25.subsystems.Lights;

/**
 * Pivot subsystem controlling the robot's pivot arm mechanism.
 * <p>
 * Manages motor control via Motion Magic Expo voltage with encoder feedback,
 * state-based position presets for coral scoring levels (L1-L4), algae removal,
 * processor/net scoring, and climb positions. Includes up/down ratchet servo control
 * and dynamic gravity feedforward compensation based on wrist and elevator positions.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author JiuJiu Liu, <a href="https://github.com/jooj99">GitHub</a>
 * @author Aneesh Terani, <a href="https://github.com/aterani">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 * @author Leo Minton, <a href="https://github.com/leo-minton">GitHub</a>
 */
public class Pivot extends SubsystemBase {
    /** Motion Magic profile enumeration for switching between normal and slow velocity profiles. */
    public enum MotionMagicProfile {
        /** Normal speed profile. */
        NORMAL(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0)
                .withMotionMagicExpo_kV(Constants.PivotConstants.EXPO_V)
                .withMotionMagicExpo_kA(Constants.PivotConstants.EXPO_A)),
        /** Slow speed profile for climb and precise movements. */
        SLOW(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0)
                .withMotionMagicExpo_kV(Constants.PivotConstants.EXPO_V_SLOW)
                .withMotionMagicExpo_kA(Constants.PivotConstants.EXPO_A));

        /** The Motion Magic configuration for this profile. */
        final MotionMagicConfigs config;

        /**
         * Constructs a MotionMagicProfile with the given configuration.
         *
         * @param config The Motion Magic configuration.
         */
        MotionMagicProfile(MotionMagicConfigs config) {
            this.config = config;
        }
    }

    /** Pivot state enumeration defining target positions for all scoring and operational modes. */
    public enum State {
        /** Manual control state - no automatic positioning. */
        MANUAL(Constants.PivotConstants.LOWER_LIMIT),
        /** Stow position - fully retracted for travel. */
        STOW(Constants.PivotConstants.STOW),
        /** Stow position for L2-L3-L4 scoring transitions. */
        L2_L3_L4_STOW(Constants.PivotConstants.L2_L3_L4_STOW),
        /** Perpendicular (90 degree) position. */
        PERPENDICULAR(90.0),
        /** Coral station intake position. */
        CORAL_STATION(Constants.PivotConstants.CORAL_STATION),
        /** Coral station intake position with obstruction. */
        CORAL_STATION_OBSTRUCTED(Constants.PivotConstants.CORAL_STATION_OBSTRUCTED),
        /** Ground-level coral collection position. */
        GROUND_CORAL(Constants.PivotConstants.GROUND_CORAL),
        /** Ground-level algae collection position. */
        GROUND_ALGAE(Constants.PivotConstants.GROUND_ALGAE),
        /** L1 coral scoring position (lowest level). */
        L1_CORAL(Constants.PivotConstants.L1_CORAL),
        /** L2 coral scoring at branch position. */
        L2_CORAL_AT_BRANCH(Constants.PivotConstants.L2_CORAL_AT_BRANCH),
        /** L2 coral scoring one coral from branch position. */
        L2_CORAL_ONE_CORAL_FROM_BRANCH(Constants.PivotConstants.L2_CORAL_ONE_CORAL_FROM_BRANCH),
        /** L3 coral scoring at branch position. */
        L3_CORAL_AT_BRANCH(Constants.PivotConstants.L3_CORAL_AT_BRANCH),
        /** L3 coral scoring one coral from branch position. */
        L3_CORAL_ONE_CORAL_FROM_BRANCH(Constants.PivotConstants.L3_CORAL_ONE_CORAL_FROM_BRANCH),
        /** L4 coral scoring at branch position (highest level). */
        L4_CORAL_AT_BRANCH(Constants.PivotConstants.L4_CORAL_AT_BRANCH),
        /** L4 coral scoring one coral from branch position. */
        L4_CORAL_ONE_CORAL_FROM_BRANCH(Constants.PivotConstants.L4_CORAL_ONE_CORAL_FROM_BRANCH),
        /** Low reef algae removal position. */
        LOW_REEF_ALGAE(Constants.PivotConstants.LOW_REEF_ALGAE),
        /** High reef algae removal position. */
        HIGH_REEF_ALGAE(Constants.PivotConstants.HIGH_REEF_ALGAE),
        /** Processor scoring position. */
        PROCESSOR(Constants.PivotConstants.PROCESSOR),
        /** Net scoring position. */
        NET(Constants.PivotConstants.NET),
        /** Prepare for climbing position. */
        PREPARE_CLIMB(Constants.PivotConstants.PREPARE_CLIMB),
        /** Climb position. */
        CLIMB(Constants.PivotConstants.CLIMB);

        /** The target pivot angle (degrees) for this state. */
        private final double position;

        /**
         * Constructs a State with a target position.
         *
         * @param position The target pivot angle in degrees.
         */
        State(double position) {
            this.position = position;
        }
    }

    /** Ratchet state enumeration for the up/down ratchet servos. */
    public enum RatchetState {
        /** Ratchet engaged - pivot can move. */
        ON(Constants.PivotConstants.DOWN_RATCHET_ON),
        /** Ratchet disengaged - pivot is locked. */
        OFF(Constants.PivotConstants.DOWN_RATCHET_OFF);

        /** The servo pulse width for this ratchet state. */
        private final int pulseWidth;

        /**
         * Constructs a RatchetState with the given pulse width.
         *
         * @param pulseWidth The servo pulse width in microseconds.
         */
        RatchetState(int pulseWidth) {
            this.pulseWidth = pulseWidth;
        }
    }

    /** The current operational state of the pivot. */
    private State currentState;

    /** The current motion magic profile (normal or slow). */
    private MotionMagicProfile currentMotionMagicProfile;

    /** The TalonFX motor controller for the pivot arm. */
    private final TalonFX pivot;

    /** The TalonFX motor controller for the cage intake. */
    private final TalonFX intake;

    /** The up ratchet servo channel on the servo hub. */
    private final ServoChannel upRatchet;

    /** The down ratchet servo channel on the servo hub. */
    private final ServoChannel downRatchet;

    /** The Motion Magic Expo voltage control request. */
    private final MotionMagicExpoVoltage request;

    /** Calculator for dynamic gravity feedforward gains based on arm, wrist, and elevator positions. */
    private final GravityGainsCalculator gravityGainsCalculator = new GravityGainsCalculator(
            Constants.PivotConstants.AXIS_POSITION,
            Constants.WristConstants.AXIS_POSITION,
            Constants.WristConstants.AXIS_TO_ZERO_COM,
            Constants.ElevatorConstants.ZERO_UPRIGHT_COM,
            Constants.ElevatorConstants.COM_TO_STAGE_2_RATIO,
            Constants.ElevatorConstants.STAGE_3_LIMIT,
            Constants.ElevatorConstants.COM_TO_STAGE_3_RATIO,
            Constants.ElevatorConstants.MASS_LBS,
            Constants.WristConstants.MASS_LBS,
            Constants.PivotConstants.G
    );

    /** The absolute error between current position and target. */
    private double error;

    /** The current dynamically-calculated gravity feedforward gain. */
    private double currentG;

    /** The last target position set during manual control. */
    private double lastManualPosition;

    /** Whether the cage intake is being manually activated. */
    private boolean cageIntakeOverride;

    /** Whether the up ratchet latch is activated (for climb). */
    private boolean activateUpLatch;

    /** Telemetry publisher for pivot state. */
    private final PivotTelemetry pivotTelemetry = new PivotTelemetry(this);

    /**
     * Constructs the Pivot subsystem.
     * <p>
     * Initializes the pivot and intake TalonFX motors with remote CANcoder feedback,
     * configures up/down ratchet servos on the REV ServoHub, and starts in the STOW state.
     */
    public Pivot() {
        currentState = State.STOW;
        currentMotionMagicProfile = MotionMagicProfile.NORMAL;

        CANcoder encoder = new CANcoder(Constants.PivotConstants.ENCODER_ID);
        encoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(Constants.PivotConstants.ENCODER_INVERT)
                        .withMagnetOffset(Constants.PivotConstants.ENCODER_ABSOLUTE_OFFSET)));

        pivot = new TalonFX(Constants.PivotConstants.LEAD_ID);
        pivot.getConfigurator().apply(new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withRemoteCANcoder(encoder)
                        .withSensorToMechanismRatio(Constants.PivotConstants.SENSOR_TO_DEGREE_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.PivotConstants.MOTOR_INVERT)
                        .withNeutralMode(Constants.PivotConstants.NEUTRAL_MODE))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.PivotConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true))
                .withSlot0(new Slot0Configs()
                        .withKV(Constants.PivotConstants.V)
                        .withKA(Constants.PivotConstants.A)
                        .withKP(Constants.PivotConstants.P)
                        .withKI(Constants.PivotConstants.I)
                        .withKD(Constants.PivotConstants.D))
                .withMotionMagic(currentMotionMagicProfile.config));

        try (TalonFX pivot2 = new TalonFX(Constants.PivotConstants.FOLLOWER_ID)) {
            pivot2.setControl(new Follower(Constants.PivotConstants.LEAD_ID, true));
        }

        intake = new TalonFX(Constants.PivotConstants.INTAKE_ID);
        intake.getConfigurator().apply(new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs())
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.PivotConstants.INTAKE_MOTOR_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.PivotConstants.CURRENT_LIMIT))
        );

        upRatchet = Constants.SERVO_HUB.getServoChannel(Constants.PivotConstants.UP_RATCHET_CHANNEL);
        upRatchet.setEnabled(true);
        upRatchet.setPowered(true);

        downRatchet = Constants.SERVO_HUB.getServoChannel(Constants.PivotConstants.DOWN_RATCHET_CHANNEL);
        downRatchet.setEnabled(true);
        downRatchet.setPowered(true);

        request = new MotionMagicExpoVoltage(getTarget());

        error = 0.0;
        currentG = Constants.PivotConstants.G;
        lastManualPosition = State.STOW.position;
    }

    /**
     * Returns the stator current draw of the pivot motor.
     *
     * @return The current in amps.
     */
    public double getCurrent() {
        return pivot.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Returns the current pivot state.
     *
     * @return The current state.
     */
    public State getState() {
        return currentState;
    }

    /**
     * Returns the appropriate L2 state based on the scoring setting.
     *
     * @param mode The robot scoring setting.
     * @return The corresponding L2 pivot state.
     */
    public State getL2State(RobotPoses.Reef.RobotScoringSetting mode) {
        return switch (mode) {
            case L1, L2, AT_BRANCH -> State.L2_CORAL_AT_BRANCH;
            case ONE_CORAL_FROM_BRANCH -> State.L2_CORAL_ONE_CORAL_FROM_BRANCH;
        };
    }

    /**
     * Returns the appropriate L3 state based on the scoring setting.
     *
     * @param mode The robot scoring setting.
     * @return The corresponding L3 pivot state.
     */
    public State getL3State(RobotPoses.Reef.RobotScoringSetting mode) {
        return switch (mode) {
            case L1, L2, AT_BRANCH -> State.L3_CORAL_AT_BRANCH;
            case ONE_CORAL_FROM_BRANCH -> State.L3_CORAL_ONE_CORAL_FROM_BRANCH;
        };
    }

    /**
     * Returns the appropriate L4 state based on the scoring setting.
     *
     * @param mode The robot scoring setting.
     * @return The corresponding L4 pivot state.
     */
    public State getL4State(RobotPoses.Reef.RobotScoringSetting mode) {
        return switch (mode) {
            case L1, L2, AT_BRANCH -> State.L4_CORAL_AT_BRANCH;
            case ONE_CORAL_FROM_BRANCH -> State.L4_CORAL_ONE_CORAL_FROM_BRANCH;
        };
    }

    /**
     * Returns the appropriate coral scoring state based on obstruction status.
     *
     * @param isObstructed Whether the scoring location is obstructed by another coral.
     * @return The corresponding pivot state.
     */
    public State getCoralScoringObstructedState(boolean isObstructed) {
        return switch (currentState) {
            case L2_CORAL_AT_BRANCH -> isObstructed ? State.L2_CORAL_ONE_CORAL_FROM_BRANCH : State.L2_CORAL_AT_BRANCH;
            case L3_CORAL_AT_BRANCH -> isObstructed ? State.L3_CORAL_ONE_CORAL_FROM_BRANCH : State.L3_CORAL_AT_BRANCH;
            case L4_CORAL_AT_BRANCH -> isObstructed ? State.L4_CORAL_ONE_CORAL_FROM_BRANCH : State.L4_CORAL_AT_BRANCH;
            default -> currentState;
        };
    }

    /**
     * Returns the up ratchet state. Disengages (OFF) during climb to allow pivot movement.
     *
     * @return The up ratchet state.
     */
    public RatchetState getUpRatchetState() {
        return getState() == State.CLIMB ? RatchetState.OFF : RatchetState.ON;
    }

    /**
     * Returns the down ratchet state. Always ON to prevent unwanted downward movement.
     *
     * @return The down ratchet state (always ON).
     */
    public RatchetState getDownRatchetState() {
        return RatchetState.ON;
    }

    /**
     * Returns the current pivot angle in degrees.
     *
     * @return The pivot position in degrees.
     */
    public double getPosition() {
        return pivot.getPosition().getValueAsDouble();
    }

    /**
     * Returns the target position for the current state.
     *
     * @return The target angle in degrees.
     */
    public double getTarget() {
        return getState() == State.MANUAL ? lastManualPosition : getState().position;
    }

    /**
     * Returns the pulse width to set for the up ratchet servo.
     *
     * @return The pulse width value based on latch state.
     */
    public int getUpRatchetPulseWidth() {
        return activateUpLatch ? RatchetState.OFF.pulseWidth : RatchetState.ON.pulseWidth;
    }

    /**
     * Returns the pulse width to set for the down ratchet servo (always ON).
     *
     * @return The ON pulse width value.
     */
    public int getDownRatchetPulseWidth() {
        return RatchetState.ON.pulseWidth;
    }

    /**
     * Checks if the pivot is at a valid start position (near stow) for safe operation.
     *
     * @return True if the pivot is within safe tolerance of the stow position.
     */
    public boolean validStartPosition() {
        return Math.abs(getPosition() - Constants.PivotConstants.STOW) <= Constants.PivotConstants.SAFE_TOLERANCE;
    }

    /**
     * Returns the absolute error between current position and target.
     *
     * @return The position error in degrees.
     */
    public double getError() {
        return error;
    }

    /**
     * Returns the current dynamically-calculated gravity feedforward gain.
     *
     * @return The gravity gain value.
     */
    public double getCurrentGravityGains() {
        return currentG;
    }

    /**
     * Returns the current pulse width of the up ratchet servo.
     *
     * @return The servo pulse width in microseconds.
     */
    public double getUpRatchetStateValue() {
        return upRatchet.getPulseWidth();
    }

    /**
     * Returns the current pulse width of the down ratchet servo.
     *
     * @return The servo pulse width in microseconds.
     */
    public double getDownRatchetStateValue() {
        return downRatchet.getPulseWidth();
    }

    /**
     * Checks if the pivot is at the specified state position.
     *
     * @param state The state to check against.
     * @return True if within tolerance of the state position.
     */
    public boolean isAtState(State state) {
        return Math.abs(state.position - getPosition()) < Constants.PivotConstants.AT_TARGET_TOLERANCE;
    }

    /**
     * Checks if the pivot is near the target position (within safe tolerance).
     *
     * @return True if near the target.
     */
    public boolean nearTarget() {
        return error < Constants.PivotConstants.SAFE_TOLERANCE;
    }

    /**
     * Checks if the pivot is at the target position (within tolerance).
     *
     * @return True if at the target.
     */
    public boolean isAtTarget() {
        return error < Constants.PivotConstants.AT_TARGET_TOLERANCE;
    }

    /**
     * Checks if moving to the target state would require passing through the stow position.
     *
     * @param state The target state to check.
     * @return True if the pivot must cross stow to reach the target.
     */
    public boolean goingThroughStow(State state) {
        return (state.position - State.STOW.position) * (getPosition() - State.STOW.position) < 0;
    }

    /** Applies the normal motion magic profile to the pivot motor. */
    public void setNormalMotionMagicProfile() {
        currentMotionMagicProfile = MotionMagicProfile.NORMAL;
        pivot.getConfigurator().apply(currentMotionMagicProfile.config);
    }

    /** Applies the slow motion magic profile to the pivot motor (used during climb). */
    public void setSlowMotionMagicProfile() {
        currentMotionMagicProfile = MotionMagicProfile.SLOW;
        pivot.getConfigurator().apply(currentMotionMagicProfile.config);
    }

    /**
     * Sets the current pivot state and manages the up ratchet latch.
     *
     * @param newState The new state to set.
     */
    private void setState(State newState) {
        activateUpLatch = newState == State.MANUAL ? activateUpLatch : newState == State.CLIMB;
        currentState = newState;
    }

    /** Sets the pivot to manual control mode, recording the current position. */
    public void setManualState() {
        setState(State.MANUAL);
        lastManualPosition = getPosition();
    }

    /** Sets the pivot to the stow position. */
    public void setStowState() {
        setState(State.STOW);
    }

    /** Sets the pivot to the L2-L3-L4 stow position. */
    public void setL2L3L4StowState() {
        setState(State.L2_L3_L4_STOW);
    }

    /** Sets the pivot to the perpendicular (90 degree) position. */
    public void setPerpendicularState() {
        setState(State.PERPENDICULAR);
    }

    /** Sets the pivot to the coral station intake position. */
    public void setCoralStationState() {
        setState(State.CORAL_STATION);
    }

    /** Sets the pivot to the obstructed coral station intake position. */
    public void setCoralStationObstructedState() {
        setState(State.CORAL_STATION_OBSTRUCTED);
    }

    /** Sets the pivot to the ground coral collection position. */
    public void setGroundCoralState() {
        setState(State.GROUND_CORAL);
    }

    /** Sets the pivot to the ground algae collection position. */
    public void setGroundAlgaeState() {
        setState(State.GROUND_ALGAE);
    }

    /** Sets the pivot to the L1 coral scoring position. */
    public void setL1CoralState() {
        setState(State.L1_CORAL);
    }

    /**
     * Sets the pivot to the L2 coral scoring position.
     *
     * @param mode The scoring setting (at branch or one coral from branch).
     */
    public void setL2CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L2_CORAL_AT_BRANCH);
            case L2, ONE_CORAL_FROM_BRANCH -> setState(State.L2_CORAL_ONE_CORAL_FROM_BRANCH);
        }
    }

    /**
     * Sets the pivot to the L3 coral scoring position.
     *
     * @param mode The scoring setting (at branch or one coral from branch).
     */
    public void setL3CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L3_CORAL_AT_BRANCH);
            case L2, ONE_CORAL_FROM_BRANCH -> setState(State.L3_CORAL_ONE_CORAL_FROM_BRANCH);
        }
    }

    /**
     * Sets the pivot to the L4 coral scoring position.
     *
     * @param mode The scoring setting (at branch or one coral from branch).
     */
    public void setL4CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L4_CORAL_AT_BRANCH);
            case L2, ONE_CORAL_FROM_BRANCH -> setState(State.L4_CORAL_ONE_CORAL_FROM_BRANCH);
        }
    }

    /**
     * Adjusts the pivot state if the current coral scoring location is obstructed.
     * Transitions from at-branch to one-coral-from-branch if obstructed.
     *
     * @param isObstructed Whether the scoring location is obstructed.
     */
    public void setCoralScoringObstructedState(boolean isObstructed) {
        switch (currentState) {
            case L2_CORAL_AT_BRANCH:
                if (isObstructed) {
                    setState(State.L2_CORAL_ONE_CORAL_FROM_BRANCH);
                }
                break;
            case L3_CORAL_AT_BRANCH:
                if (isObstructed) {
                    setState(State.L3_CORAL_ONE_CORAL_FROM_BRANCH);
                }
                break;
            case L4_CORAL_AT_BRANCH:
                if (isObstructed) {
                    setState(State.L4_CORAL_ONE_CORAL_FROM_BRANCH);
                }
                break;
        }
    }

    /** Sets the pivot to the low reef algae removal position. */
    public void setLowReefAlgaeState() {
        setState(State.LOW_REEF_ALGAE);
    }

    /** Sets the pivot to the high reef algae removal position. */
    public void setHighReefAlgaeState() {
        setState(State.HIGH_REEF_ALGAE);
    }

    /** Sets the pivot to the processor scoring position. */
    public void setProcessorState() {
        setState(State.PROCESSOR);
    }

    /** Sets the pivot to the net scoring position. */
    public void setNetState() {
        setState(State.NET);
    }

    /** Sets the pivot to the prepare climb position. */
    public void setPrepareClimbState() {
        setState(State.PREPARE_CLIMB);
    }

    /** Sets the pivot to the climb position and switches to slow motion profile. */
    public void setClimbState() {
        setSlowMotionMagicProfile();
        setState(State.CLIMB);
    }

    /** Activates the cage intake motor for climbing. */
    public void activateCageIntake() {
        cageIntakeOverride = true;
    }

    /** Stops the cage intake motor. */
    public void stopCageIntake() {
        cageIntakeOverride = false;
    }

    /**
     * Commands the pivot to hold its target position with dynamically-calculated gravity feedforward.
     *
     * @param elevatorPosition The current elevator position used for gravity calculation.
     * @param wristPosition The current wrist position used for gravity calculation.
     */
    public void holdTarget(double elevatorPosition, double wristPosition) {
        currentG = gravityGainsCalculator.calculateGFromPositions(getPosition(), wristPosition, elevatorPosition);
        pivot.setControl(request.withPosition(getTarget()).withFeedForward(currentG));
    }

    /**
     * Moves the pivot manually with exponential output scaling.
     * Prevents driving beyond the upper and lower limits.
     *
     * @param axisValue The joystick axis value (-1.0 to 1.0).
     */
    public void move(double axisValue) {
        pivot.set(axisValue > 0
                ? axisValue * EquationUtil.expOutput(Constants.PivotConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                : axisValue * EquationUtil.expOutput(getPosition() - Constants.PivotConstants.LOWER_LIMIT, 1, 5, 10));
    }

    /**
     * Periodically updates telemetry, calculates the absolute error between current
     * position and target, controls the LED indicator based on valid start position,
     * updates the ratchet servo positions, and runs the cage intake motor during climb.
     */
    @Override
    public void periodic() {
        pivotTelemetry.publishValues();

        error = Math.abs(getTarget() - getPosition());

        Lights.setLights((validStartPosition()) && DriverStation.isDisabled());

        upRatchet.setPulseWidth(getUpRatchetPulseWidth());
        downRatchet.setPulseWidth(getDownRatchetPulseWidth());

        if (currentState == State.PREPARE_CLIMB || cageIntakeOverride) {
            intake.set(0.6);
        } else {
            intake.set(0.0);
        }
    }
}