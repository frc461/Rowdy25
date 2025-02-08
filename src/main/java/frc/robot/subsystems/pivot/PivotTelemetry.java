package frc.robot.subsystems.pivot;

import javax.xml.crypto.dom.DOMStructure;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.constants.Constants;

public class PivotTelemetry {
    private final Pivot pivot;

    public PivotTelemetry(Pivot pivot) {
        this.pivot = pivot;
    }

    private final NetworkTable pivotTelemetryTable = Constants.NT_INSTANCE.getTable("PivotTelemetry");
    private final DoublePublisher pivotPositionPub = pivotTelemetryTable.getDoubleTopic("Pivot Position").publish();
    private final DoublePublisher pivotTargetPub = pivotTelemetryTable.getDoubleTopic("Pivot Target").publish();
    private final DoublePublisher pivotErrorPub = pivotTelemetryTable.getDoubleTopic("Pivot Error").publish();
    private final DoublePublisher pivotRachetPositionPub = pivotTelemetryTable.getDoubleTopic("Pivot Rachet Position").publish();
    private final BooleanPublisher pivotIsRatcheted = pivotTelemetryTable.getBooleanTopic("Pivot Is Ratcheted").publish();
    private final StringPublisher pivotStatePub = pivotTelemetryTable.getStringTopic("Pivot State").publish();

    public void publishValues() {
        pivotPositionPub.set(pivot.getPosition());
        pivotTargetPub.set(pivot.getTarget());
        pivotErrorPub.set(pivot.getError());
        pivotRachetPositionPub.set(pivot.getRatchetValue());
        pivotIsRatcheted.set(pivot.isRatcheted());
        pivotStatePub.set(pivot.getState().toString());

        logValues();
    }

    private void logValues() {
        DogLog.log("Pivot Position", pivot.getPosition());
        DogLog.log("PivotTarget", pivot.getTarget());
        DogLog.log("PivotError", pivot.getError());
        DogLog.log("PivotIsRatcheted", pivot.isRatcheted());
        DogLog.log("PivotState", pivot.getState().toString());
    }
}
