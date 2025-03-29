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
    private final BooleanPublisher hasAlgaePub = intakeTelemetryTable.getBooleanTopic("Intake Has Algae").publish();
    private final StringPublisher currentStatePub = intakeTelemetryTable.getStringTopic("Intake State").publish();

    public void publishValues() {
        hasCoralPub.set(intake.hasCoral());
        hasAlgaePub.set(intake.hasAlgae());
        currentStatePub.set(intake.getState().toString());

        logValues();
    }

    private void logValues() {
        DogLog.log("IntakeHasCoral", intake.hasCoral());
        DogLog.log("IntakeHasAlgae", intake.hasAlgae());
        DogLog.log("IntakeState", intake.getState());
    }
}
