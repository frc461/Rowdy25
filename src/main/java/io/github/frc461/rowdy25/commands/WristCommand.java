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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.frc461.rowdy25.RobotStates;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.subsystems.wrist.Wrist;

import java.util.function.DoubleSupplier;

public class WristCommand extends Command {
    private final Wrist wrist;
    private final DoubleSupplier manualAxisValue;
    private final DoubleSupplier pivotPosition;
    private final DoubleSupplier elevatorPosition;
    private final RobotStates robotStates;

    public WristCommand(Wrist wrist, DoubleSupplier manualAxisValue, DoubleSupplier pivotPosition, DoubleSupplier elevatorPosition, RobotStates robotStates) {
        this.wrist = wrist;
        this.manualAxisValue = manualAxisValue;
        this.pivotPosition = pivotPosition;
        this.elevatorPosition = elevatorPosition;
        this.robotStates = robotStates;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        double axisValue = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND) * 0.25;
        if (axisValue != 0.0) {
            wrist.setManualState();
            robotStates.setManualState();
            wrist.move(axisValue, pivotPosition.getAsDouble(), elevatorPosition.getAsDouble());
        } else {
            wrist.holdTarget(pivotPosition.getAsDouble());
        }
        wrist.setTarget(pivotPosition.getAsDouble(), elevatorPosition.getAsDouble());
    }
}
