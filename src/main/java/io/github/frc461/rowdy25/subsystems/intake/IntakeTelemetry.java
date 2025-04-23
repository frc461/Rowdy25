package io.github.frc461.rowdy25.subsystems.intake;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.*;
import io.github.frc461.rowdy25.constants.Constants;

public class IntakeTelemetry {
    private final Intake intake;

    public IntakeTelemetry(Intake intake) {
        this.intake = intake;
    }

    private final NetworkTable intakeTelemetryTable = Constants.NT_INSTANCE.getTable("IntakeTelemetry");
    private final DoubleArrayPublisher rgbPub = intakeTelemetryTable.getDoubleArrayTopic("RGB Canandcolor Detection").publish();
    private final BooleanPublisher hasCoralPub = intakeTelemetryTable.getBooleanTopic("Intake Has Coral").publish();
    private final BooleanPublisher beamBreakBrokenPub = intakeTelemetryTable.getBooleanTopic("Intake BeamBreak Broken").publish();
    private final BooleanPublisher hasAlgaePub = intakeTelemetryTable.getBooleanTopic("Intake Has Algae").publish();
    private final StringPublisher currentStatePub = intakeTelemetryTable.getStringTopic("Intake State").publish();
    private final DoublePublisher proximityPub = intakeTelemetryTable.getDoubleTopic("Canandcolor Proximity").publish();
    private final DoubleEntry proximityObjectDetectionThresholdEntry = intakeTelemetryTable.getDoubleTopic("Canandcolor Proximity Object Detection Threshold").getEntry(Constants.IntakeConstants.DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD);
    private final BooleanPublisher intakeStallingPub = intakeTelemetryTable.getBooleanTopic("Intake Stalling").publish();
    private final StringPublisher stallIntakePub = intakeTelemetryTable.getStringTopic("Intake Stall Intent").publish();
    private final DoublePublisher intakeCurrentPub = intakeTelemetryTable.getDoubleTopic("Intake Current").publish();

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
