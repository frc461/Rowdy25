package io.github.frc461.rowdy25.subsystems.elevator;

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
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.constants.RobotIdentity;
import io.github.frc461.rowdy25.constants.RobotPoses;
import io.github.frc461.rowdy25.util.EquationUtil;

/**
 * Elevator subsystem controlling the robot's telescoping elevator mechanism.
 * <p>
 * Manages motor control via Motion Magic Expo voltage, limit switch homing,
 * state-based position presets for coral scoring levels (L1-L4), algae removal,
 * processor/net scoring, and climb positions. Uses a TalonFX leader-follower
 * configuration with gravity feedforward compensation.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 * @author JiuJiu Liu, <a href="https://github.com/jooj99">GitHub</a>
 * @author Aneesh Terani, <a href="https://github.com/aterani">GitHub</a>
 * @author Mysterious Man, <a href="https://github.com/tidymoman">GitHub</a>
 */
public class Elevator extends SubsystemBase {
    /** Elevator state enumeration defining target positions for all scoring and operational modes. */
    public enum State {
        /** Manual control state - no automatic positioning. */
        MANUAL(Constants.ElevatorConstants.LOWER_LIMIT),
        /** Stow position - fully retracted for travel. */
        STOW(Constants.ElevatorConstants.STOW),
        /** Stow position for L2-L3-L4 scoring transitions. */
        L2_L3_L4_STOW(Constants.ElevatorConstants.L2_L3_L4_STOW),
        /** Coral station intake position. */
        CORAL_STATION(Constants.ElevatorConstants.CORAL_STATION),
        /** Coral station intake position with obstruction. */
        CORAL_STATION_OBSTRUCTED(Constants.ElevatorConstants.CORAL_STATION_OBSTRUCTED),
        /** Ground-level algae collection position. */
        GROUND_ALGAE(Constants.ElevatorConstants.GROUND_ALGAE),
        /** Ground-level coral collection position. */
        GROUND_CORAL(Constants.ElevatorConstants.GROUND_CORAL),
        /** L1 coral scoring position (lowest level). */
        L1_CORAL(Constants.ElevatorConstants.L1_CORAL),
        /** L2 coral scoring at branch position. */
        L2_CORAL_AT_BRANCH(Constants.ElevatorConstants.L2_CORAL_AT_BRANCH),
        /** L2 coral scoring one coral from branch position. */
        L2_CORAL_ONE_CORAL_FROM_BRANCH(Constants.ElevatorConstants.L2_CORAL_ONE_CORAL_FROM_BRANCH),
        /** L3 coral scoring at branch position. */
        L3_CORAL_AT_BRANCH(Constants.ElevatorConstants.L3_CORAL_AT_BRANCH),
        /** L3 coral scoring one coral from branch position. */
        L3_CORAL_ONE_CORAL_FROM_BRANCH(Constants.ElevatorConstants.L3_CORAL_ONE_CORAL_FROM_BRANCH),
        /** L4 coral scoring at branch position (highest level). */
        L4_CORAL_AT_BRANCH(Constants.ElevatorConstants.L4_CORAL_AT_BRANCH),
        /** L4 coral scoring one coral from branch position. */
        L4_CORAL_ONE_CORAL_FROM_BRANCH(Constants.ElevatorConstants.L4_CORAL_ONE_CORAL_FROM_BRANCH),
        /** Low reef algae removal position. */
        LOW_REEF_ALGAE(Constants.ElevatorConstants.LOW_REEF_ALGAE),
        /** High reef algae removal position. */
        HIGH_REEF_ALGAE(Constants.ElevatorConstants.HIGH_REEF_ALGAE),
        /** Net scoring position. */
        NET(Constants.ElevatorConstants.NET),
        /** Processor scoring position. */
        PROCESSOR(Constants.ElevatorConstants.PROCESSOR),
        /** Prepare for climbing position. */
        PREPARE_CLIMB(Constants.ElevatorConstants.PREPARE_CLIMB),
        /** Climb position. */
        CLIMB(Constants.ElevatorConstants.CLIMB);

        /** The target elevator position (inches) for this state. */
        private final double position;

        /**
         * Constructs a State with a target position.
         *
         * @param position The target elevator position in inches.
         */
        State(double position) {
            this.position = position;
        }
    }

    /** The current operational state of the elevator. */
    private State currentState;

    /** The leader TalonFX motor controller for the elevator. */
    private final TalonFX elevator;

    /** The lower limit switch for homing. */
    private final DigitalInput lowerSwitch;

    /** The Motion Magic Expo voltage control request. */
    private final MotionMagicExpoVoltage request;

    /** The absolute error between current position and target. */
    private double error;

    /** The last target position set during manual control. */
    private double lastManualPosition;

    /** Telemetry publisher for elevator state. */
    private final ElevatorTelemetry elevatorTelemetry = new ElevatorTelemetry(this);

    /**
     * Constructs the Elevator subsystem.
     * <p>
     * Initializes the leader and follower TalonFX motors with Motion Magic Expo
     * configuration, sets up the lower limit switch, and configures the elevator
     * to start in the STOW state.
     */
    public Elevator() {
        currentState = State.STOW;

        elevator = new TalonFX(Constants.ElevatorConstants.LEAD_ID);
        elevator.getConfigurator().apply(new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(Constants.ElevatorConstants.ROTOR_TO_INCH_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.ElevatorConstants.MOTOR_INVERT)
                        .withNeutralMode(Constants.ElevatorConstants.NEUTRAL_MODE))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true))
                .withSlot0(new Slot0Configs()
                        .withKV(Constants.ElevatorConstants.V)
                        .withKA(Constants.ElevatorConstants.A)
                        .withKP(Constants.ElevatorConstants.P)
                        .withKI(Constants.ElevatorConstants.I)
                        .withKD(Constants.ElevatorConstants.D))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.ElevatorConstants.EXPO_V)
                        .withMotionMagicExpo_kA(Constants.ElevatorConstants.EXPO_A)));

        try (TalonFX elevator2 = new TalonFX(Constants.ElevatorConstants.FOLLOWER_ID)) {
            elevator2.setControl(new Follower(Constants.ElevatorConstants.LEAD_ID, true));
        }

        lowerSwitch = new DigitalInput(Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_DIO_PORT);

        request = new MotionMagicExpoVoltage(0);

        elevator.setPosition(0.0);
        error = 0.0;
        lastManualPosition = State.STOW.position;
    }

    /**
     * Returns the stator current draw of the elevator motor.
     *
     * @return The current in amps.
     */
    public double getCurrent() {
        return elevator.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Returns the rotor velocity of the elevator motor.
     *
     * @return The rotor velocity.
     */
    public double getRotorVelocity() {
        return elevator.getRotorVelocity().getValueAsDouble();
    }

    /**
     * Returns the current elevator state.
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
     * @return The corresponding L2 elevator state.
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
     * @return The corresponding L3 elevator state.
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
     * @return The corresponding L4 elevator state.
     */
    public State getL4State(RobotPoses.Reef.RobotScoringSetting mode) {
        return switch (mode) {
            case L1, L2, AT_BRANCH -> State.L4_CORAL_AT_BRANCH;
            case ONE_CORAL_FROM_BRANCH -> State.L4_CORAL_ONE_CORAL_FROM_BRANCH;
        };
    }

    /**
     * Returns the appropriate coral scoring state based on obstruction status.
     * If obstructed, returns the one-coral-from-branch state; otherwise returns the at-branch state.
     *
     * @param isObstructed Whether the scoring location is obstructed by another coral.
     * @return The corresponding elevator state.
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
     * Returns the current elevator position in inches.
     *
     * @return The elevator position in inches.
     */
    public double getPosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    /**
     * Returns the target position for the current state.
     *
     * @return The target position in inches.
     */
    public double getTarget() {
        return getState() == State.MANUAL ? lastManualPosition : getState().position;
    }

    /**
     * Checks if the lower limit switch is triggered.
     *
     * @return True if the limit switch is triggered (robot is not ROWDY identity and switch is active).
     */
    public boolean lowerSwitchTriggered() {
        return Constants.IDENTITY != RobotIdentity.ROWDY && !lowerSwitch.get();
    }

    /**
     * Checks if the elevator is at the specified state position.
     *
     * @param state The state to check against.
     * @return True if within tolerance of the state position.
     */
    public boolean isAtState(State state) {
        return Math.abs(state.position - getPosition()) < Constants.ElevatorConstants.AT_TARGET_TOLERANCE;
    }

    /**
     * Checks if the elevator is near the target position (within safe tolerance).
     *
     * @return True if near the target.
     */
    public boolean nearTarget() {
        return error < Constants.ElevatorConstants.SAFE_TOLERANCE;
    }

    /**
     * Checks if the elevator is at the target position (within tolerance).
     *
     * @return True if at the target.
     */
    public boolean isAtTarget() {
        return error < Constants.ElevatorConstants.AT_TARGET_TOLERANCE;
    }

    /**
     * Checks if the elevator is currently moving downward toward the target state.
     *
     * @param state The target state to check against.
     * @return True if the current position is above or at the target state position.
     */
    public boolean goingDown(State state) {
        return getPosition() >= state.position;
    }

    /**
     * Sets the current elevator state.
     *
     * @param state The new state to set.
     */
    private void setState(State state) {
        currentState = state;
    }

    /** Sets the elevator to manual control mode, recording the current position. */
    public void setManualState() {
        setState(State.MANUAL);
        lastManualPosition = getPosition();
    }

    /** Sets the elevator to the stow position. */
    public void setStowState() {
        setState(State.STOW);
    }

    /** Sets the elevator to the L2-L3-L4 stow position. */
    public void setL2L3L4StowState() {
        setState(State.L2_L3_L4_STOW);
    }

    /** Sets the elevator to the coral station intake position. */
    public void setCoralStationState() {
        setState(State.CORAL_STATION);
    }

    /** Sets the elevator to the obstructed coral station intake position. */
    public void setCoralStationObstructedState() {
        setState(State.CORAL_STATION_OBSTRUCTED);
    }

    /** Sets the elevator to the ground coral collection position. */
    public void setGroundCoralState() {
        setState(State.GROUND_CORAL);
    }

    /** Sets the elevator to the ground algae collection position. */
    public void setGroundAlgaeState() {
        setState(State.GROUND_ALGAE);
    }

    /** Sets the elevator to the L1 coral scoring position. */
    public void setL1CoralState() {
        setState(State.L1_CORAL);
    }

    /**
     * Sets the elevator to the L2 coral scoring position.
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
     * Sets the elevator to the L3 coral scoring position.
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
     * Sets the elevator to the L4 coral scoring position.
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
     * Adjusts the elevator state if the current coral scoring location is obstructed.
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

    /** Sets the elevator to the low reef algae removal position. */
    public void setLowReefAlgaeState() {
        setState(State.LOW_REEF_ALGAE);
    }

    /** Sets the elevator to the high reef algae removal position. */
    public void setHighReefAlgaeState() {
        setState(State.HIGH_REEF_ALGAE);
    }

    /** Sets the elevator to the processor scoring position. */
    public void setProcessorState() {
        setState(State.PROCESSOR);
    }

    /** Sets the elevator to the net scoring position. */
    public void setNetState() {
        setState(State.NET);
    }

    /** Sets the elevator to the prepare climb position. */
    public void setPrepareClimbState() {
        setState(State.PREPARE_CLIMB);
    }

    /** Sets the elevator to the climb position. */
    public void setClimbState() {
        setState(State.CLIMB);
    }

    /** Checks and calibrates the elevator position using the limit switch if triggered. */
    public void checkLimitSwitch() {
        if (lowerSwitchTriggered() || (!lowerSwitchTriggered() && getPosition() <= Constants.ElevatorConstants.LOWER_LIMIT)) {
            elevator.setPosition(Constants.ElevatorConstants.LOWER_LIMIT);
        }
    }

    /**
     * Commands the elevator to hold its target position with gravity feedforward.
     *
     * @param pivotPosition The current pivot position used to calculate gravity gains.
     */
    public void holdTarget(double pivotPosition) {
        checkLimitSwitch();
        elevator.setControl(request.withPosition(getTarget()).withFeedForward(Constants.ElevatorConstants.G.apply(pivotPosition)));
    }

    /**
     * Moves the elevator manually with exponential output scaling.
     * Prevents driving beyond the upper and lower limits.
     *
     * @param axisValue The joystick axis value (-1.0 to 1.0).
     */
    public void move(double axisValue) {
        checkLimitSwitch();
        elevator.set(axisValue > 0
                ? axisValue * EquationUtil.expOutput(Constants.ElevatorConstants.UPPER_LIMIT - getPosition(), 1, 0.5, 10)
                : axisValue * EquationUtil.expOutput(getPosition() - Constants.ElevatorConstants.LOWER_LIMIT, 1, 0.5, 10));
    }

    /**
     * Periodically updates telemetry and calculates the absolute error between
     * the current position and the target position.
     */
    @Override
    public void periodic() {
        elevatorTelemetry.publishValues();

        error = Math.abs(getPosition() - getTarget());
    }
}