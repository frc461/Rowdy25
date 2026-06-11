package io.github.frc461.rowdy25.subsystems.wrist;

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
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.constants.RobotPoses;
import io.github.frc461.rowdy25.util.EquationUtil;

/**
 * Wrist subsystem controlling the robot's wrist mechanism.
 * <p>
 * Manages motor control via Motion Magic Expo voltage with encoder feedback,
 * state-based position presets for coral scoring levels (L1-L4), algae removal,
 * processor/net scoring, and climb positions. The wrist target is dynamically
 * clamped based on elevator and pivot positions to prevent collisions.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Leo Minton, <a href="https://github.com/leo-minton">GitHub</a>
 * @author JiuJiu Liu, <a href="https://github.com/jooj99">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 * @author Aneesh Terani, <a href="https://github.com/aterani">GitHub</a>
 */
public class Wrist extends SubsystemBase {
    /** Wrist state enumeration defining target positions for all scoring and operational modes. */
    public enum State {
        /** Manual control state - no automatic positioning. */
        MANUAL(Constants.WristConstants.LOWER_LIMIT.apply(0.0, 50.0)),
        /** Stow position - fully retracted for travel. */
        STOW(Constants.WristConstants.STOW),
        /** Stow position for L2-L3-L4 scoring transitions. */
        L2_L3_L4_STOW(Constants.WristConstants.L2_L3_L4_STOW),
        /** Coral station intake position. */
        CORAL_STATION(Constants.WristConstants.CORAL_STATION),
        /** Coral station intake position with obstruction. */
        CORAL_STATION_OBSTRUCTED(Constants.WristConstants.CORAL_STATION_OBSTRUCTED),
        /** Ground-level coral collection position. */
        GROUND_CORAL(Constants.WristConstants.GROUND_CORAL),
        /** Ground-level algae collection position. */
        GROUND_ALGAE(Constants.WristConstants.GROUND_ALGAE),
        /** L1 coral scoring position (lowest level). */
        L1_CORAL(Constants.WristConstants.L1_CORAL),
        /** L2 coral scoring at branch position. */
        L2_CORAL_AT_BRANCH(Constants.WristConstants.L2_CORAL_AT_BRANCH),
        /** L2 coral scoring one coral from branch position. */
        L2_CORAL_ONE_CORAL_FROM_BRANCH(Constants.WristConstants.L2_CORAL_ONE_CORAL_FROM_BRANCH),
        /** L3 coral scoring at branch position. */
        L3_CORAL_AT_BRANCH(Constants.WristConstants.L3_CORAL_AT_BRANCH),
        /** L3 coral scoring one coral from branch position. */
        L3_CORAL_ONE_CORAL_FROM_BRANCH(Constants.WristConstants.L3_CORAL_ONE_CORAL_FROM_BRANCH),
        /** L4 coral scoring at branch position (highest level). */
        L4_CORAL_AT_BRANCH(Constants.WristConstants.L4_CORAL_AT_BRANCH),
        /** L4 coral scoring one coral from branch position. */
        L4_CORAL_ONE_CORAL_FROM_BRANCH(Constants.WristConstants.L4_CORAL_ONE_CORAL_FROM_BRANCH),
        /** Low reef algae removal position. */
        LOW_REEF_ALGAE(Constants.WristConstants.LOW_REEF_ALGAE),
        /** High reef algae removal position. */
        HIGH_REEF_ALGAE(Constants.WristConstants.HIGH_REEF_ALGAE),
        /** Processor scoring position. */
        PROCESSOR(Constants.WristConstants.PROCESSOR),
        /** Net scoring position. */
        NET(Constants.WristConstants.NET),
        /** Prepare for climbing position. */
        PREPARE_CLIMB(Constants.WristConstants.PREPARE_CLIMB),
        /** Climb position. */
        CLIMB(Constants.WristConstants.CLIMB);

        /** The target wrist angle (degrees) for this state. */
        private final double position;

        /**
         * Constructs a State with a target position.
         *
         * @param position The target wrist angle in degrees.
         */
        State(double position) {
            this.position = position;
        }
    }


    /** The current operational state of the wrist. */
    private State currentState;

    /** The TalonFX motor controller for the wrist. */
    private final TalonFX wrist;

    /** The Motion Magic Expo voltage control request. */
    private final MotionMagicExpoVoltage request;

    /** The current target position (degrees), clamped by physical limits. */
    private double target;

    /** The absolute error between current position and target. */
    private double error;

    /** The last target position set during manual control. */
    private double lastManualPosition;

    /** Telemetry publisher for wrist state. */
    private final WristTelemetry wristTelemetry = new WristTelemetry(this);

    /**
     * Constructs the Wrist subsystem.
     * <p>
     * Initializes the TalonFX motor with remote CANcoder feedback and Motion Magic
     * Expo configuration. Configures the wrist to start in the STOW state.
     */
    public Wrist() {
        currentState = State.STOW;

        CANcoder encoder = new CANcoder(Constants.WristConstants.ENCODER_ID);
        encoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(Constants.WristConstants.ENCODER_INVERT)
                        .withMagnetOffset(Constants.WristConstants.ENCODER_ABSOLUTE_OFFSET)));

        wrist = new TalonFX(Constants.WristConstants.MOTOR_ID);
        wrist.getConfigurator().apply(new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withRemoteCANcoder(encoder)
                        .withSensorToMechanismRatio(Constants.WristConstants.SENSOR_TO_DEGREE_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.WristConstants.MOTOR_INVERT)
                        .withNeutralMode(Constants.WristConstants.NEUTRAL_MODE))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.WristConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true))
                .withSlot0(new Slot0Configs()
                        .withKV(Constants.WristConstants.V)
                        .withKA(Constants.WristConstants.A)
                        .withKP(Constants.WristConstants.P)
                        .withKI(Constants.WristConstants.I)
                        .withKD(Constants.WristConstants.D))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.WristConstants.EXPO_V)
                        .withMotionMagicExpo_kA(Constants.WristConstants.EXPO_A)));

        request = new MotionMagicExpoVoltage(0);

        target = State.STOW.position;
        error = 0.0;
        lastManualPosition = State.STOW.position;
    }

    /**
     * Returns the stator current draw of the wrist motor.
     *
     * @return The current in amps.
     */
    public double getCurrent() {
        return wrist.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Returns the current wrist state.
     *
     * @return The current state.
     */
    public State getState() {
        return currentState;
    }

    /**
     * Returns the current target position.
     *
     * @return The target position in degrees.
     */
    public double getTarget() {
        return target;
    }

    /**
     * Returns the current wrist position in degrees.
     *
     * @return The wrist position in degrees.
     */
    public double getPosition() {
        return wrist.getPosition().getValueAsDouble();
    }

    /**
     * Returns the absolute error between the current position and target.
     *
     * @return The position error in degrees.
     */
    public double getError() {
        return error;
    }

    /**
     * Checks if the wrist is at the specified state position.
     *
     * @param state The state to check against.
     * @return True if within tolerance of the state position.
     */
    public boolean isAtState(State state) {
        return Math.abs(state.position - getPosition()) < Constants.WristConstants.AT_TARGET_TOLERANCE;
    }

    /**
     * Checks if the wrist is near the target position (within safe tolerance).
     *
     * @return True if near the target.
     */
    public boolean nearTarget() {
        return error < Constants.WristConstants.SAFE_TOLERANCE;
    }

    /**
     * Checks if the wrist is at the target position (within tolerance).
     *
     * @return True if at the target.
     */
    public boolean isAtTarget() {
        return error < Constants.WristConstants.AT_TARGET_TOLERANCE;
    }

    /**
     * Sets the target position, clamped by physical limits based on elevator and pivot positions.
     *
     * @param pivotPosition The current pivot angle used for lower limit calculation.
     * @param elevatorPosition The current elevator height used for limit calculation.
     */
    public void setTarget(double pivotPosition, double elevatorPosition) {
        this.target = MathUtil.clamp(
                getState() == State.MANUAL ? lastManualPosition : getState().position,
                Constants.WristConstants.LOWER_LIMIT.apply(elevatorPosition, pivotPosition),
                Constants.WristConstants.UPPER_LIMIT.apply(elevatorPosition)
        );
    }

    /**
     * Sets the current wrist state.
     *
     * @param newState The new state to set.
     */
    private void setState(State newState) {
        currentState = newState;
    }

    /** Sets the wrist to manual control mode, recording the current position. */
    public void setManualState() {
        setState(State.MANUAL);
        lastManualPosition = getPosition();
    }

    /** Sets the wrist to the stow position. */
    public void setStowState() {
        setState(State.STOW);
    }

    /** Sets the wrist to the L2-L3-L4 stow position. */
    public void setL2L3L4StowState() {
        setState(State.L2_L3_L4_STOW);
    }

    /** Sets the wrist to the coral station intake position. */
    public void setCoralStationState() {
        setState(State.CORAL_STATION);
    }

    /** Sets the wrist to the obstructed coral station intake position. */
    public void setCoralStationObstructedState() {
        setState(State.CORAL_STATION_OBSTRUCTED);
    }

    /** Sets the wrist to the ground coral collection position. */
    public void setGroundCoralState() {
        setState(State.GROUND_CORAL);
    }

    /** Sets the wrist to the ground algae collection position. */
    public void setGroundAlgaeState() {
        setState(State.GROUND_ALGAE);
    }

    /** Sets the wrist to the L1 coral scoring position. */
    public void setL1CoralState() {
        setState(State.L1_CORAL);
    }

    /**
     * Sets the wrist to the L2 coral scoring position.
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
     * Sets the wrist to the L3 coral scoring position.
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
     * Sets the wrist to the L4 coral scoring position.
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
     * Adjusts the wrist state if the current coral scoring location is obstructed.
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

    /** Sets the wrist to the low reef algae removal position. */
    public void setLowReefAlgaeState() {
        setState(State.LOW_REEF_ALGAE);
    }

    /** Sets the wrist to the high reef algae removal position. */
    public void setHighReefAlgaeState() {
        setState(State.HIGH_REEF_ALGAE);
    }

    /** Sets the wrist to the processor scoring position. */
    public void setProcessorState() {
        setState(State.PROCESSOR);
    }

    /** Sets the wrist to the net scoring position. */
    public void setNetState() {
        setState(State.NET);
    }

    /** Sets the wrist to the prepare climb position. */
    public void setPrepareClimbState() {
        setState(State.PREPARE_CLIMB);
    }

    /** Sets the wrist to the climb position. */
    public void setClimbState() {
        setState(State.CLIMB);
    }

    /**
     * Commands the wrist to hold its target position with gravity feedforward.
     *
     * @param pivotPosition The current pivot angle used to calculate gravity gains.
     */
    public void holdTarget(double pivotPosition) {
        wrist.setControl(request.withPosition(target).withFeedForward(Constants.WristConstants.G.apply(getPosition(), pivotPosition)));
    }

    /**
     * Moves the wrist manually with exponential output scaling.
     * Prevents driving beyond the dynamic upper and lower limits.
     *
     * @param axisValue The joystick axis value (-1.0 to 1.0).
     * @param pivotPosition The current pivot angle for lower limit calculation.
     * @param elevatorPosition The current elevator height for upper limit calculation.
     */
    public void move(double axisValue, double pivotPosition, double elevatorPosition) {
        wrist.set(axisValue > 0
                ? axisValue * EquationUtil.expOutput(Constants.WristConstants.UPPER_LIMIT.apply(elevatorPosition) - getPosition(), 1, 5, 10)
                : axisValue * EquationUtil.expOutput(getPosition() - Constants.WristConstants.LOWER_LIMIT.apply(elevatorPosition, pivotPosition), 1, 5, 10));
    }

    /**
     * Periodically updates wrist telemetry and internal state.
     *
     * <p>Publishes telemetry via {@link WristTelemetry} and updates the cached
     * absolute error between the commanded target and the current wrist position.
     * This method is called on the main robot loop and should remain efficient.
     */
    @Override
    public void periodic() {
        wristTelemetry.publishValues();

        error = Math.abs(target - getPosition());
    }
}