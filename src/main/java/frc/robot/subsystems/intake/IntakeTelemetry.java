package frc.robot.subsystems.intake;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.*;
import frc.robot.constants.Constants;

public class IntakeTelemetry {
    private final Intake intake;

    public IntakeTelemetry(Intake intake) {
        this.intake = intake;
    }

    private final NetworkTable intakeTelemetryTable = Constants.NT_INSTANCE.getTable("IntakeTelemetry");
    private final BooleanPublisher hasCoralPub = intakeTelemetryTable.getBooleanTopic("Intake Has Coral").publish();
    private final BooleanPublisher beamBreakBrokenPub = intakeTelemetryTable.getBooleanTopic("Intake BeamBreak Broken").publish();
    private final BooleanPublisher hasAlgaePub = intakeTelemetryTable.getBooleanTopic("Intake Has Algae").publish();
    private final StringPublisher currentStatePub = intakeTelemetryTable.getStringTopic("Intake State").publish();
    private final DoublePublisher proximityObjectDetectionThreshold = intakeTelemetryTable.getDoubleTopic("Canandcolor Proximity Object Detection Threshold").publish();
    private final DoubleSubscriber setProximityObjectDetectionThreshold = intakeTelemetryTable.getDoubleTopic("Canandcolor Proximity Object Detection Threshold").subscribe(Constants.IntakeConstants.DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD);

    public void publishValues() {
        hasCoralPub.set(intake.hasCoral());
        beamBreakBrokenPub.set(intake.beamBreakBroken());
        hasAlgaePub.set(intake.hasAlgae());
        currentStatePub.set(intake.getState().toString());
        proximityObjectDetectionThreshold.set(setProximityObjectDetectionThreshold.get(Constants.IntakeConstants.DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD));
        intake.setProximityObjectDetectionThreshold.accept(setProximityObjectDetectionThreshold.get(Constants.IntakeConstants.DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD));

        logValues();
    }

    private void logValues() {
        DogLog.log("IntakeHasCoral", intake.hasCoral());
        DogLog.log("IntakeBeamBreakBroken", intake.beamBreakBroken());
        DogLog.log("IntakeHasAlgae", intake.hasAlgae());
        DogLog.log("IntakeState", intake.getState());
    }
}
