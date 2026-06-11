package io.github.frc461.rowdy25.subsystems.intake;

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
import edu.wpi.first.networktables.*;
import io.github.frc461.rowdy25.constants.Constants;

/**
 * Telemetry publisher for the intake subsystem.
 * <p>
 * Publishes intake state (RGB readings, coral/beam break/algae status, proximity, stall info, current)
 * to NetworkTables and DogLog.
 *
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class IntakeTelemetry {
    /** The intake subsystem to read state from. */
    private final Intake intake;

    /**
     * Constructs an IntakeTelemetry instance.
     *
     * @param intake The intake subsystem to read state from.
     */
    public IntakeTelemetry(Intake intake) {
        this.intake = intake;
    }

    /** NetworkTable for intake telemetry data. */
    private final NetworkTable intakeTelemetryTable = Constants.NT_INSTANCE.getTable("IntakeTelemetry");

    /** Publisher for RGB color readings from the Canandcolor sensor. */
    private final DoubleArrayPublisher rgbPub = intakeTelemetryTable.getDoubleArrayTopic("RGB Canandcolor Detection").publish();

    /** Publisher for whether the intake has coral. */
    private final BooleanPublisher hasCoralPub = intakeTelemetryTable.getBooleanTopic("Intake Has Coral").publish();

    /** Publisher for whether the beam break sensor is broken. */
    private final BooleanPublisher beamBreakBrokenPub = intakeTelemetryTable.getBooleanTopic("Intake BeamBreak Broken").publish();

    /** Publisher for whether the intake has algae. */
    private final BooleanPublisher hasAlgaePub = intakeTelemetryTable.getBooleanTopic("Intake Has Algae").publish();

    /** Publisher for the current intake state name. */
    private final StringPublisher currentStatePub = intakeTelemetryTable.getStringTopic("Intake State").publish();

    /** Publisher for the Canandcolor proximity sensor reading. */
    private final DoublePublisher proximityPub = intakeTelemetryTable.getDoubleTopic("Canandcolor Proximity").publish();

    /** NetworkTable entry for live tuning the proximity detection threshold. */
    private final DoubleEntry proximityObjectDetectionThresholdEntry = intakeTelemetryTable.getDoubleTopic("Canandcolor Proximity Object Detection Threshold").getEntry(Constants.IntakeConstants.DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD);

    /** Publisher for whether the intake motor is stalling. */
    private final BooleanPublisher intakeStallingPub = intakeTelemetryTable.getBooleanTopic("Intake Stalling").publish();

    /** Publisher for the intake stall intent (coral vs algae). */
    private final StringPublisher stallIntakePub = intakeTelemetryTable.getStringTopic("Intake Stall Intent").publish();

    /** Publisher for the intake motor current draw. */
    private final DoublePublisher intakeCurrentPub = intakeTelemetryTable.getDoubleTopic("Intake Current").publish();

    /**
     * Publishes all intake telemetry values to NetworkTables and DogLog.
     * <p>
     * Also reads the proximity detection threshold entry from NetworkTables
     * and applies it to the intake for live tuning.
     */
    public void publishValues() {
        rgbPub.set(intake.getColorReading());
        hasCoralPub.set(intake.hasCoral());
        beamBreakBrokenPub.set(intake.beamBreakBroken());
        hasAlgaePub.set(intake.hasAlgae());
        currentStatePub.set(intake.getState().toString());
        proximityPub.set(intake.getProximity());
        proximityObjectDetectionThresholdEntry.set(proximityObjectDetectionThresholdEntry.get()); // TODO SHOP: TEST ENTRY
        intake.setProximityObjectDetectionThreshold.accept(proximityObjectDetectionThresholdEntry.get());
        intakeStallingPub.set(intake.hasAlgaeOrCoralStuck.getAsBoolean());
        stallIntakePub.set(intake.stallIntent.name());
        intakeCurrentPub.set(intake.getCurrent());

        logValues();
    }

    /** Logs intake state values to DogLog. */
    private void logValues() {
        DogLog.log("IntakeRGBReading", intake.getColorReading());
        DogLog.log("IntakeHasCoral", intake.hasCoral());
        DogLog.log("IntakeBeamBreakBroken", intake.beamBreakBroken());
        DogLog.log("IntakeHasAlgae", intake.hasAlgae());
        DogLog.log("IntakeState", intake.getState());
        DogLog.log("IntakeStalling", intake.hasAlgaeOrCoralStuck.getAsBoolean());
        DogLog.log("IntakeCanandcolorProximity", intake.getProximity());
    }
}