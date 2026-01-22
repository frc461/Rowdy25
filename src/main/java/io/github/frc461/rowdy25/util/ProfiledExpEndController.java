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

public class ProfiledExpEndController {
    private double lastTimestamp;
    private Function<Double, Double> controllerWithExpEnd;

    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public ProfiledExpEndController(Function<Double, Double> controllerWithExpEnd, TrapezoidProfile.Constraints constraints) {
        this.controllerWithExpEnd = controllerWithExpEnd;
        this.constraints = constraints;
        profile = new TrapezoidProfile(this.constraints);
    }

    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    public void updateController(Function<Double, Double> controllerWithExpEnd) {
        this.controllerWithExpEnd = controllerWithExpEnd;
    }

    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        this.constraints = constraints;
        profile = new TrapezoidProfile(this.constraints);
    }

    public void setGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
    }

    public double calculate(double currentPosition, double currentTimestamp) {

        double thisPeriod = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;

        setpoint = profile.calculate(thisPeriod, setpoint, goal);
        return controllerWithExpEnd.apply(Math.abs(setpoint.position - currentPosition));
    }

    public double calculate(double currentPosition, TrapezoidProfile.State targetState, double currentTimestamp) {
        setGoal(targetState);
        return calculate(currentPosition, currentTimestamp);
    }

    public double calculate(double currentPosition, double targetPosition, double currentTimestamp) {
        setGoal(new TrapezoidProfile.State(targetPosition, 0));
        return calculate(currentPosition, currentTimestamp);
    }

    public double calculate(double currentPosition, TrapezoidProfile.Constraints constraints, double currentTimestamp) {
        setConstraints(constraints);
        return calculate(currentPosition, currentTimestamp);
    }

    public double calculate(
            double currentPosition,
            TrapezoidProfile.State goal,
            TrapezoidProfile.Constraints constraints,
            double currentTimestamp
    ) {
        setConstraints(constraints);
        return calculate(currentPosition, goal, currentTimestamp);
    }

    public void reset(TrapezoidProfile.State currentState, double timestamp) {
        setpoint = currentState;
        lastTimestamp = timestamp;
    }

    // In the case of the goal being set to 0, currentPosition is the current error and the currentVelocity is the change in error
    public void reset(double currentPosition, double currentVelocity, double timestamp) {
        reset(new TrapezoidProfile.State(currentPosition, currentVelocity), timestamp);
    }

    public void reset(double currentPosition, double timestamp) {
        reset(currentPosition, 0.0, timestamp);
    }
}
