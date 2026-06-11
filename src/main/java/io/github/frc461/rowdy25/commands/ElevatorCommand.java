package io.github.frc461.rowdy25.commands;

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

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.frc461.rowdy25.RobotStates;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.subsystems.elevator.Elevator;

/**
 * A command that provides manual and automatic control of the elevator subsystem.
 * <p>
 * When the manual axis is active (beyond deadband), the elevator is moved directly
 * and the robot states are set to manual mode. Otherwise, the elevator holds its
 * current target position based on the pivot position.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 */
public class ElevatorCommand extends Command {
    /** The elevator subsystem. */
    private final Elevator elevator;

    /** Supplier for the manual control axis value. */
    private final DoubleSupplier manualAxisValue;

    /** Supplier for the current pivot position used to determine elevator hold target. */
    private final DoubleSupplier pivotPosition;

    /** The robot states manager for tracking manual/auto mode transitions. */
    private final RobotStates robotStates;

    /**
     * Constructs an ElevatorCommand.
     *
     * @param elevator The elevator subsystem.
     * @param manualAxisValue Supplier for the manual control axis value.
     * @param pivotPosition Supplier for the current pivot position.
     * @param robotStates The robot states manager.
     */
    public ElevatorCommand(Elevator elevator, DoubleSupplier manualAxisValue, DoubleSupplier pivotPosition, RobotStates robotStates) {
        this.elevator = elevator;
        this.manualAxisValue = manualAxisValue;
        this.pivotPosition = pivotPosition;
        this.robotStates = robotStates;
        addRequirements(elevator);
    }

    /**
     * Executes the command's control logic every 20ms.
     * <p>
     * If the manual axis exceeds the deadband, the elevator enters manual state
     * and moves at a reduced rate. Otherwise, the elevator holds its target
     * position based on the current pivot position.
     */
    @Override
    public void execute() {
        double axisValue = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND) * 0.25;
        if (axisValue != 0.0) {
            elevator.setManualState();
            robotStates.setManualState();
            elevator.move(axisValue);
        } else {
            elevator.holdTarget(pivotPosition.getAsDouble());
        }
    }
}