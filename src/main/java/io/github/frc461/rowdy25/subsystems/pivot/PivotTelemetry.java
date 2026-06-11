package io.github.frc461.rowdy25.subsystems.pivot;

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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import io.github.frc461.rowdy25.constants.Constants;

/**
 * Telemetry publisher for the pivot subsystem.
 * <p>
 * Publishes pivot state (position, target, error, gravity gains, ratchet status, current)
 * to NetworkTables and DogLog.
 *
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class PivotTelemetry {
    /** The pivot subsystem to read state from. */
    private final Pivot pivot;

    /**
     * Constructs a PivotTelemetry instance.
     *
     * @param pivot The pivot subsystem to read state from.
     */
    public PivotTelemetry(Pivot pivot) {
        this.pivot = pivot;
    }

    /** NetworkTable for pivot telemetry data. */
    private final NetworkTable pivotTelemetryTable = Constants.NT_INSTANCE.getTable("PivotTelemetry");

    /** Publisher for the pivot position (degrees). */
    private final DoublePublisher pivotPositionPub = pivotTelemetryTable.getDoubleTopic("Pivot Position").publish();

    /** Publisher for the pivot target position (degrees). */
    private final DoublePublisher pivotTargetPub = pivotTelemetryTable.getDoubleTopic("Pivot Target").publish();

    /** Publisher for the pivot position error. */
    private final DoublePublisher pivotErrorPub = pivotTelemetryTable.getDoubleTopic("Pivot Error").publish();

    /** Publisher for the current pivot state name. */
    private final StringPublisher pivotStatePub = pivotTelemetryTable.getStringTopic("Pivot State").publish();

    /** Publisher for the current gravity feedforward gain. */
    private final DoublePublisher pivotGravityGainsPub = pivotTelemetryTable.getDoubleTopic("Pivot Gravity Gains").publish();

    /** Publisher for the up ratchet servo pulse width. */
    private final DoublePublisher pivotUpRatchetPositionPub = pivotTelemetryTable.getDoubleTopic("Pivot Up Ratchet Position").publish();

    /** Publisher for the up ratchet state (ON/OFF). */
    private final StringPublisher pivotIsUpRatcheted = pivotTelemetryTable.getStringTopic("Pivot Up Ratchet State").publish();

    /** Publisher for the down ratchet servo pulse width. */
    private final DoublePublisher pivotDownRatchetPositionPub = pivotTelemetryTable.getDoubleTopic("Pivot Down Ratchet Position").publish();

    /** Publisher for the down ratchet state (ON/OFF). */
    private final StringPublisher pivotIsDownRatcheted = pivotTelemetryTable.getStringTopic("Pivot Down Ratchet State").publish();

    /** Publisher for whether the pivot is at its target position. */
    private final BooleanPublisher pivotAtTargetPub = pivotTelemetryTable.getBooleanTopic("Pivot At Target").publish();

    /** Publisher for whether the pivot is near its target position. */
    private final BooleanPublisher pivotNearTargetPub = pivotTelemetryTable.getBooleanTopic("Pivot Near Target").publish();

    /** Publisher for the pivot motor current draw. */
    private final DoublePublisher pivotCurrent = pivotTelemetryTable.getDoubleTopic("Pivot Current").publish();

    /**
     * Publishes all pivot telemetry values to NetworkTables and DogLog.
     */
    public void publishValues() {
        pivotPositionPub.set(pivot.getPosition());
        pivotTargetPub.set(pivot.getTarget());
        pivotErrorPub.set(pivot.getError());
        pivotStatePub.set(pivot.getState().name());
        pivotGravityGainsPub.set(pivot.getCurrentGravityGains());
        pivotUpRatchetPositionPub.set(pivot.getUpRatchetStateValue());
        pivotIsUpRatcheted.set(pivot.getUpRatchetState().name());
        pivotDownRatchetPositionPub.set(pivot.getDownRatchetStateValue());
        pivotIsDownRatcheted.set(pivot.getDownRatchetState().name());
        pivotAtTargetPub.set(pivot.isAtTarget());
        pivotNearTargetPub.set(pivot.nearTarget());
        pivotCurrent.set(pivot.getCurrent());

        logValues();
    }

    /** Logs pivot state values to DogLog. */
    private void logValues() {
        DogLog.log("PivotPose", pivot.getPosition());
        DogLog.log("PivotTarget", pivot.getTarget());
        DogLog.log("PivotError", pivot.getError());
        DogLog.log("PivotState", pivot.getState().name());
        DogLog.log("PivotGravityGains", pivot.getCurrentGravityGains());
        DogLog.log("PivotUpRatchetPosition", pivot.getUpRatchetStateValue());
        DogLog.log("PivotRatchetedState", pivot.getUpRatchetState());
        DogLog.log("PivotDownRatchetPosition", pivot.getDownRatchetStateValue());
        DogLog.log("PivotDownRatchetedState", pivot.getDownRatchetState());
        DogLog.log("PivotIsAtTarget", pivot.isAtTarget());
        DogLog.log("PivotNearTarget", pivot.nearTarget());
    }
}