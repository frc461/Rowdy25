package frc.robot.subsystems.intake;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.constants.Constants;

public class IntakeTelemetry {
    private final Intake intake;

    public IntakeTelemetry(Intake intake) {
        this.intake = intake;
    }

    private final NetworkTable intakeTelemetryTable = Constants.NT_INSTANCE.getTable("IntakeTelemetry");
    private final DoubleArrayPublisher rgbPub = intakeTelemetryTable.getDoubleArrayTopic("RGB Canandcolor Detection").publish();
    private final BooleanPublisher hasCoralPub = intakeTelemetryTable.getBooleanTopic("Intake Has Coral").publish();
    private final BooleanPublisher hasAlgaePub = intakeTelemetryTable.getBooleanTopic("Intake Has Algae").publish();
    private final StringPublisher currentStatePub = intakeTelemetryTable.getStringTopic("Intake State").publish();
    private final DoublePublisher proximityPub = intakeTelemetryTable.getDoubleTopic("Canandcolor Proximity").publish();

    public void publishValues() {
        rgbPub.set(intake.getColorReading());
        hasCoralPub.set(intake.hasCoral());
        hasAlgaePub.set(intake.hasAlgae());
        currentStatePub.set(intake.getCurrentState().toString());
        proximityPub.set(intake.getProximity());

        logValues();
    }

    private void logValues() {
        DogLog.log("IntakeRGBReading", intake.getColorReading());
        DogLog.log("IntakeHasCoral", intake.hasCoral());
        DogLog.log("IntakeHasAlgae", intake.hasAlgae());
        DogLog.log("IntakeState", intake.getCurrentState());
        DogLog.log("IntakeCanandcolorProximity", intake.getProximity());
    }
}
