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

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * An exponential/function-based controller with trapezoidal motion profiling.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class ProfiledExpEndController {
    /** The timestamp of the last calculation */
    private double lastTimestamp;
    /** The function-based controller */
    private Function<Double, Double> controllerWithExpEnd;

    /** The motion profile constraints */
    private TrapezoidProfile.Constraints constraints;
    /** The motion profile */
    private TrapezoidProfile profile;
    /** The goal state of the motion profile */
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    /** The current setpoint of the motion profile */
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    /**
     * Creates a new ProfiledExpEndController with the given function-based controller and motion profile constraints.
     *
     * @param controllerWithExpEnd The function-based controller that takes in the absolute error and returns the control output
     * @param constraints The motion profile constraints
     */
    public ProfiledExpEndController(Function<Double, Double> controllerWithExpEnd, TrapezoidProfile.Constraints constraints) {
        this.controllerWithExpEnd = controllerWithExpEnd;
        this.constraints = constraints;
        profile = new TrapezoidProfile(this.constraints);
    }

    /**
     * Gets the current setpoint of the motion profile.
     *
     * @return The current setpoint of the motion profile
     */
    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    /**
     * Updates the function-based controller.
     *
     * @param controllerWithExpEnd The new function-based controller
     */
    public void updateController(Function<Double, Double> controllerWithExpEnd) {
        this.controllerWithExpEnd = controllerWithExpEnd;
    }

    /**
     * Sets the motion profile constraints.
     *
     * @param constraints The new motion profile constraints
     */
    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        this.constraints = constraints;
        profile = new TrapezoidProfile(this.constraints);
    }

    /**
     * Sets the goal state of the motion profile.
     *
     * @param goal The new goal state of the motion profile
     */
    public void setGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
    }

    /**
     * Calculates the control output based on the current position and timestamp.
     *
     * @param currentPosition The current position
     * @param currentTimestamp The current timestamp
     * @return The control output
     */
    public double calculate(double currentPosition, double currentTimestamp) {

        double thisPeriod = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;

        setpoint = profile.calculate(thisPeriod, setpoint, goal);
        return controllerWithExpEnd.apply(Math.abs(setpoint.position - currentPosition));
    }

    /**
     * Calculates the control output based on the current position, target state, and timestamp.
     *
     * @param currentPosition The current position
     * @param targetState The target state
     * @param currentTimestamp The current timestamp
     * @return The control output
     */
    public double calculate(double currentPosition, TrapezoidProfile.State targetState, double currentTimestamp) {
        setGoal(targetState);
        return calculate(currentPosition, currentTimestamp);
    }

    /**
     * Calculates the control output based on the current position, target position, and timestamp.
     *
     * @param currentPosition The current position
     * @param targetPosition The target position
     * @param currentTimestamp The current timestamp
     * @return The control output
     */
    public double calculate(double currentPosition, double targetPosition, double currentTimestamp) {
        setGoal(new TrapezoidProfile.State(targetPosition, 0));
        return calculate(currentPosition, currentTimestamp);
    }

    /**
     * Calculates the control output based on the current position, motion profile constraints, and timestamp.
     *
     * @param currentPosition The current position
     * @param constraints The motion profile constraints
     * @param currentTimestamp The current timestamp
     * @return The control output
     */
    public double calculate(double currentPosition, TrapezoidProfile.Constraints constraints, double currentTimestamp) {
        setConstraints(constraints);
        return calculate(currentPosition, currentTimestamp);
    }

    /**
     * Calculates the control output based on the current position, target state, motion profile constraints, and timestamp.
     *
     * @param currentPosition The current position
     * @param goal The target state
     * @param constraints The motion profile constraints
     * @param currentTimestamp The current timestamp
     * @return The control output
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
     * Resets the controller with the given current state and timestamp.
     *
     * @param currentState The current state
     * @param timestamp The current timestamp
     */
    public void reset(TrapezoidProfile.State currentState, double timestamp) {
        setpoint = currentState;
        lastTimestamp = timestamp;
    }

    /**
     * Resets the controller with the given current position, velocity, and timestamp.
     *
     * <p> In the case of the goal being set to 0, currentPosition is the current error and the currentVelocity is the change in error.
     *
     * @param currentPosition The current position
     * @param currentVelocity The current velocity
     * @param timestamp The current timestamp
     */
    public void reset(double currentPosition, double currentVelocity, double timestamp) {
        reset(new TrapezoidProfile.State(currentPosition, currentVelocity), timestamp);
    }

    /**
     * Resets the controller with the given current position and timestamp.
     *
     * <p> In the case of the goal being set to 0, currentPosition is the current error.
     *
     * @param currentPosition The current position
     * @param timestamp The current timestamp
     */
    public void reset(double currentPosition, double timestamp) {
        reset(currentPosition, 0.0, timestamp);
    }
}
