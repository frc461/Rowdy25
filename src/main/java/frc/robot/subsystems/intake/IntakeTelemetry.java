package frc.robot.subsystems.intake;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.constants.Constants;

public class IntakeTelemetry {
    private final Intake intake;

    public IntakeTelemetry(Intake intake) {
        this.intake = intake;
    }

    private final NetworkTable IntakeTelemetryTable = Constants.NT_INSTANCE.getTable("IntakeTelemetry");
    private final BooleanPublisher hasCoralPub = IntakeTelemetryTable.getBooleanTopic("Intake Has Coral").publish();
    private final BooleanPublisher hasAlgaePub = IntakeTelemetryTable.getBooleanTopic("Intake Has Algae").publish();
    private final BooleanPublisher hasGamePiecePub = IntakeTelemetryTable.getBooleanTopic("Intake Has Game Piece").publish();

    public void publishValues() {
        hasCoralPub.set(intake.hasCoral());
        hasAlgaePub.set(intake.hasAlgae());
        hasGamePiecePub.set(intake.hasGamePiece());

        logValues();
    }

    private void logValues() {
        //DogLog.log("IntakeRGBReading", intake.getColorReading());
        //DogLog.log("IntakeHasCoral", intake.hasCoral());
        //DogLog.log("IntakeHasAlgae", intake.hasAlgae());
    }
}
