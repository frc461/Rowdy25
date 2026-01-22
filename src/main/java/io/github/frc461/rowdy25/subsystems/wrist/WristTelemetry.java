package io.github.frc461.rowdy25.subsystems.wrist;

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

public class WristTelemetry {
    private final Wrist wrist;

    public WristTelemetry(Wrist wrist) {
        this.wrist = wrist;
    }

    private final NetworkTable wristTelemetryTable = Constants.NT_INSTANCE.getTable("WristTelemetry");
    private final DoublePublisher wristPositionPub = wristTelemetryTable.getDoubleTopic("Wrist Position").publish();
    private final DoublePublisher wristTargetPub = wristTelemetryTable.getDoubleTopic("Wrist Target").publish();
    private final DoublePublisher wristErrorPub = wristTelemetryTable.getDoubleTopic("Wrist Error").publish();
    private final StringPublisher wristStatePub = wristTelemetryTable.getStringTopic("Wrist State").publish();
    private final BooleanPublisher wristAtTargetPub = wristTelemetryTable.getBooleanTopic("Wrist At Target").publish();
    private final BooleanPublisher wristNearTargetPub = wristTelemetryTable.getBooleanTopic("Wrist Near Target").publish();
    private final DoublePublisher wristCurrentPub = wristTelemetryTable.getDoubleTopic("Wrist Current").publish();

    public void publishValues() {
        wristPositionPub.set(wrist.getPosition());
        wristTargetPub.set(wrist.getTarget());
        wristErrorPub.set(wrist.getError());
        wristStatePub.set(wrist.getState().toString());
        wristAtTargetPub.set(wrist.isAtTarget());
        wristNearTargetPub.set(wrist.nearTarget());
        wristCurrentPub.set(wrist.getCurrent());

        logValues();
    }

    private void logValues() {
        DogLog.log("WristPosition", wrist.getPosition());
        DogLog.log("WristTarget", wrist.getTarget());
        DogLog.log("WristError", wrist.getError());
        DogLog.log("WristState", wrist.getState().toString());
        DogLog.log("WristIsAtTarget", wrist.isAtTarget());
        DogLog.log("WristNearTarget", wrist.nearTarget());
    }
}
