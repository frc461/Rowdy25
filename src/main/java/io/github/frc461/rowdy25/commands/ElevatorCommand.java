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

public class ElevatorCommand extends Command {
    private final Elevator elevator;
    private final DoubleSupplier manualAxisValue;
    private final DoubleSupplier pivotPosition;
    private final RobotStates robotStates;

    public ElevatorCommand(Elevator elevator, DoubleSupplier manualAxisValue, DoubleSupplier pivotPosition, RobotStates robotStates) {
        this.elevator = elevator;
        this.manualAxisValue = manualAxisValue;
        this.pivotPosition = pivotPosition;
        this.robotStates = robotStates;
        addRequirements(elevator);
    }

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
