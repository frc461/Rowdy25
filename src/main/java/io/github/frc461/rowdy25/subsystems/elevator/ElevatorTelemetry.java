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

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import io.github.frc461.rowdy25.constants.Constants;

/**
 * Telemetry publisher for the elevator subsystem.
 * <p>
 * Publishes elevator state (position, target, state name, current, limit switch)
 * to NetworkTables and DogLog.
 *
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class ElevatorTelemetry {
    /** The elevator subsystem to read state from. */
    private final Elevator elevator;

    /**
     * Constructs an ElevatorTelemetry instance.
     *
     * @param elevator The elevator subsystem to read state from.
     */
    public ElevatorTelemetry(Elevator elevator) {
        this.elevator = elevator;
    }

    /** NetworkTable for elevator telemetry data. */
    private final NetworkTable elevatorTelemetryTable = Constants.NT_INSTANCE.getTable("ElevatorTelemetry");

    /** Publisher for the elevator position in inches. */
    private final DoublePublisher elevatorPositionInchesPub = elevatorTelemetryTable.getDoubleTopic("Elevator Position (in)").publish();

    /** Publisher for the elevator position in meters. */
    private final DoublePublisher elevatorPositionMetersPub = elevatorTelemetryTable.getDoubleTopic("Elevator Position (m)").publish();

    /** Publisher for the elevator target position. */
    private final DoublePublisher elevatorTargetPub = elevatorTelemetryTable.getDoubleTopic("Elevator Target").publish();

    /** Publisher for the current elevator state name. */
    private final StringPublisher elevatorStatePub = elevatorTelemetryTable.getStringTopic("Elevator State").publish();

    /** Publisher for whether the elevator is at its target position. */
    private final BooleanPublisher elevatorAtTargetPub = elevatorTelemetryTable.getBooleanTopic("Elevator At Target").publish();

    /** Publisher for whether the elevator is near its target position. */
    private final BooleanPublisher elevatorNearTargetPub = elevatorTelemetryTable.getBooleanTopic("Elevator Near Target").publish();

    /** Publisher for whether the lower limit switch is triggered. */
    private final BooleanPublisher elevatorSwitchTriggered = elevatorTelemetryTable.getBooleanTopic("Elevator Limit Switch Triggered").publish();

    /** Publisher for the elevator motor current draw. */
    private final DoublePublisher elevatorCurrentPub = elevatorTelemetryTable.getDoubleTopic("Elevator Current").publish();

    /** Publisher for the elevator rotor velocity. */
    private final DoublePublisher elevatorRotorVelocityPub = elevatorTelemetryTable.getDoubleTopic("Elevator Rotor Velocity").publish();

    /**
     * Publishes all elevator telemetry values to NetworkTables and DogLog.
     */
    public void publishValues() {
        elevatorPositionInchesPub.set(elevator.getPosition());
        elevatorPositionMetersPub.set(Units.inchesToMeters(elevator.getPosition()));
        elevatorTargetPub.set(elevator.getTarget());
        elevatorStatePub.set(elevator.getState().name());
        elevatorAtTargetPub.set(elevator.isAtTarget());
        elevatorNearTargetPub.set(elevator.nearTarget());
        elevatorSwitchTriggered.set(elevator.lowerSwitchTriggered());
        elevatorCurrentPub.set(elevator.getCurrent());
        elevatorRotorVelocityPub.set(elevator.getRotorVelocity());

        logValues();
    }

    /** Logs elevator state values to DogLog. */
    private void logValues() {
        DogLog.log("ElevatorPosition", elevator.getPosition());
        DogLog.log("ElevatorTarget", elevator.getTarget());
        DogLog.log("ElevatorState", elevator.getState());
        DogLog.log("ElevatorIsAtTarget", elevator.isAtTarget());
        DogLog.log("ElevatorNearTarget", elevator.nearTarget());
        DogLog.log("ElevatorSwitchTriggered", elevator.lowerSwitchTriggered());
    }
}
