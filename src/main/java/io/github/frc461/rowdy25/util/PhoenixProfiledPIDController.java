package io.github.frc461.rowdy25.util;

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

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * A Phoenix-based PID controller with trapezoidal motion profiling.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class PhoenixProfiledPIDController {
    /** The internal PID controller used for calculations. */
    private final PhoenixPIDController controller;
    /** The minimum input value for continuous input handling. */
    private double minInput;
    /** The maximum input value for continuous input handling. */
    private double maxInput;
    /** The timestamp of the last calculation. */
    private double lastTimestamp;

    /** The motion profile constraints. */
    private TrapezoidProfile.Constraints constraints;
    /** The motion profile used to generate setpoints. */
    private TrapezoidProfile profile;
    /** The goal state for the motion profile. */
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    /** The current setpoint state from the motion profile. */
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    /**
     * Constructs a PhoenixProfiledPIDController with specified PID gains and motion profile constraints.
     *
     * @param Kp The proportional gain.
     * @param Ki The integral gain.
     * @param Kd The derivative gain.
     * @param constraints The motion profile constraints.
     */
    public PhoenixProfiledPIDController(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
        controller = new PhoenixPIDController(Kp, Ki, Kd);
        this.constraints = constraints;
        profile = new TrapezoidProfile(this.constraints);
    }

    /**
     * Gets the current setpoint state.
     *
     * @return The current setpoint state.
     */
    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    /**
     * Checks if the controller is at the setpoint.
     *
     * @return True if at setpoint, false otherwise.
     */
    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    /**
     * Checks if the controller has reached the goal state.
     *
     * @return True if at goal, false otherwise.
     */
    public boolean atGoal() {
        return atSetpoint() && goal.equals(setpoint);
    }

    /**
     * Sets new motion profile constraints.
     *
     * @param constraints The new motion profile constraints.
     */
    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        this.constraints = constraints;
        profile = new TrapezoidProfile(this.constraints);
    }

    /**
     * Sets the PID gains for the controller.
     *
     * @param Kp The proportional gain.
     * @param Ki The integral gain.
     * @param Kd The derivative gain.
     */
    public void setPID(double Kp, double Ki, double Kd) {
        controller.setPID(Kp, Ki, Kd);
    }

    /**
     * Sets the I-Zone for the controller. This is the time threshold after which the integral term is active.
     *
     * @param iZone The I-Zone value.
     */
    public void setIZone(double iZone) {
        controller.setIZone(iZone);
    }

    /**
     * Sets the goal state for the motion profile.
     *
     * @param goal The goal state.
     */
    public void setGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
    }

    /**
     * Enables continuous input handling for the controller.
     *
     * @param minimumInput The minimum input value.
     * @param maximumInput The maximum input value.
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        controller.enableContinuousInput(minimumInput, maximumInput);
        minInput = minimumInput;
        maxInput = maximumInput;
    }

    /**
     * Disables continuous input handling for the controller.
     */
    public void disableContinuousInput() {
        controller.disableContinuousInput();
    }

    /**
     * Sets the integrator range for the controller. The integrator value is capped or minimized in kI.
     *
     * @param minimumIntegral The minimum integral value.
     * @param maximumIntegral The maximum integral value.
     */
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        controller.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    /**
     * Sets the positional tolerance for the controller to consider itself at the setpoint.
     *
     * @param positionalTolerance The positional tolerance.
     */
    public void setTolerance(double positionalTolerance) {
        controller.setTolerance(positionalTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Calculates the control output based on the current position and timestamp.
     *
     * @param currentPosition The current position.
     * @param currentTimestamp The current timestamp.
     * @return The control output.
     */
    public double calculate(double currentPosition, double currentTimestamp) {
        if (controller.isContinuousInputEnabled()) {
            // Get error which is the smallest distance between goal and measurement
            double errorBound = (maxInput - minInput) / 2.0;
            double goalMinDistance = MathUtil.inputModulus(goal.position - currentPosition, -errorBound, errorBound);
            double setpointMinDistance =
                    MathUtil.inputModulus(setpoint.position - currentPosition, -errorBound, errorBound);

            // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
            // may be outside the input range after this operation, but that's OK because the controller
            // will still go there and report an error of zero. In other words, the setpoint only needs to
            // be offset from the measurement by the input range modulus; they don't need to be equal.
            goal.position = goalMinDistance + currentPosition;
            setpoint.position = setpointMinDistance + currentPosition;
        }

        double thisPeriod = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;

        setpoint = profile.calculate(thisPeriod, setpoint, goal);
        return controller.calculate(currentPosition, setpoint.position, currentTimestamp);
    }

    /**
     * Calculates the control output based on the current position, target state, and timestamp.
     *
     * @param currentPosition The current position.
     * @param targetState The target state.
     * @param currentTimestamp The current timestamp.
     * @return The control output.
     */
    public double calculate(double currentPosition, TrapezoidProfile.State targetState, double currentTimestamp) {
        setGoal(targetState);
        return calculate(currentPosition, currentTimestamp);
    }

    /**
     * Calculates the control output based on the current position, target position, and timestamp.
     *
     * @param currentPosition The current position.
     * @param targetPosition The target position.
     * @param currentTimestamp The current timestamp.
     * @return The control output.
     */
    public double calculate(double currentPosition, double targetPosition, double currentTimestamp) {
        setGoal(new TrapezoidProfile.State(targetPosition, 0));
        return calculate(currentPosition, currentTimestamp);
    }

    /**
     * Calculates the control output based on the current position, motion profile constraints, and timestamp.
     *
     * @param currentPosition The current position.
     * @param constraints The motion profile constraints.
     * @param currentTimestamp The current timestamp.
     * @return The control output.
     */
    public double calculate(double currentPosition, TrapezoidProfile.Constraints constraints, double currentTimestamp) {
        setConstraints(constraints);
        return calculate(currentPosition, currentTimestamp);
    }

    /**
     * Calculates the control output based on the current position, target state, motion profile constraints, and timestamp.
     *
     * @param currentPosition The current position.
     * @param goal The target state.
     * @param constraints The motion profile constraints.
     * @param currentTimestamp The current timestamp.
     * @return The control output.
     */
    public double calculate(
            double currentPosition,
            TrapezoidProfile.State goal,
            TrapezoidProfile.Constraints constraints,
            double currentTimestamp
    ) {
        setConstraints(constraints);
        return calculate(currentPosition, goal, currentTimestamp);
    }

    /**
     * Resets the controller with the specified current state and timestamp.
     *
     * @param currentState The current state.
     * @param timestamp The current timestamp.
     */
    public void reset(TrapezoidProfile.State currentState, double timestamp) {
        controller.reset();
        setpoint = currentState;
        lastTimestamp = timestamp;
    }

    /**
     * Resets the controller with the specified current position, velocity, and timestamp.
     *
     * <p> In the case of the goal being set to 0, currentPosition is the current error and the currentVelocity is the change in error.
     *
     * @param currentPosition The current position.
     * @param currentVelocity The current velocity.
     * @param timestamp The current timestamp.
     */
    public void reset(double currentPosition, double currentVelocity, double timestamp) {
        reset(new TrapezoidProfile.State(currentPosition, currentVelocity), timestamp);
    }

    /**
     * Resets the controller with the specified current position and timestamp, assuming zero velocity.
     *
     * @param currentPosition The current position.
     * @param timestamp The current timestamp.
     */
    public void reset(double currentPosition, double timestamp) {
        reset(currentPosition, 0.0, timestamp);
    }
}
