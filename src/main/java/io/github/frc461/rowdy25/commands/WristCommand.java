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

/**
 * A command that provides manual and automatic control of the wrist subsystem.
 * <p>
 * When the manual axis is active (beyond deadband), the wrist is moved directly
 * and the robot states are set to manual mode. Otherwise, the wrist holds its
 * target position based on the current pivot position. Additionally, the wrist
 * target is updated every cycle based on the pivot and elevator positions.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Leo Minton, <a href="https://github.com/leo-minton">GitHub</a>
 */
public class WristCommand extends Command {
    /** The wrist subsystem. */
    private final Wrist wrist;

    /** Supplier for the manual control axis value. */
    private final DoubleSupplier manualAxisValue;

    /** Supplier for the current pivot position used to determine wrist hold target. */
    private final DoubleSupplier pivotPosition;

    /** Supplier for the current elevator position used to determine wrist hold target. */
    private final DoubleSupplier elevatorPosition;

    /** The robot states manager for tracking manual/auto mode transitions. */
    private final RobotStates robotStates;

    /**
     * Constructs a WristCommand.
     *
     * @param wrist The wrist subsystem.
     * @param manualAxisValue Supplier for the manual control axis value.
     * @param pivotPosition Supplier for the current pivot position.
     * @param elevatorPosition Supplier for the current elevator position.
     * @param robotStates The robot states manager.
     */
    public WristCommand(Wrist wrist, DoubleSupplier manualAxisValue, DoubleSupplier pivotPosition, DoubleSupplier elevatorPosition, RobotStates robotStates) {
        this.wrist = wrist;
        this.manualAxisValue = manualAxisValue;
        this.pivotPosition = pivotPosition;
        this.elevatorPosition = elevatorPosition;
        this.robotStates = robotStates;
        addRequirements(wrist);
    }

    /**
     * Executes the command's control logic every 20ms.
     * <p>
     * If the manual axis exceeds the deadband, the wrist enters manual state
     * and moves at a reduced rate. Otherwise, the wrist holds its target
     * position based on the current pivot position. The wrist target is updated
     * each cycle based on the pivot and elevator positions for proper coordinated
     * motion.
     */
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