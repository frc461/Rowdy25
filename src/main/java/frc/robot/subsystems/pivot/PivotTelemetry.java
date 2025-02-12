package frc.robot.subsystems.pivot;

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
    private final DoublePublisher pivotRatchetPositionPub = pivotTelemetryTable.getDoubleTopic("Pivot Ratchet Position").publish();
    private final StringPublisher pivotIsRatcheted = pivotTelemetryTable.getStringTopic("Pivot Ratchet State").publish();
    private final BooleanPublisher pivotAtTargetPub = pivotTelemetryTable.getBooleanTopic("Pivot At Target").publish();
    private final BooleanPublisher pivotNearTargetPub = pivotTelemetryTable.getBooleanTopic("Pivot Near Target").publish();

    public void publishValues() {
        pivotPositionPub.set(pivot.getPosition());
        pivotTargetPub.set(pivot.getTarget());
        pivotErrorPub.set(pivot.getError());
        pivotRatchetPositionPub.set(pivot.getRatchetStateValue());
        pivotIsRatcheted.set(pivot.getRatchetState().name());
        pivotAtTargetPub.set(pivot.isAtTarget());
        pivotNearTargetPub.set(pivot.nearTarget());

        logValues();
    }

    private void logValues() {
        DogLog.log("PivotPose", pivot.getPosition());
        DogLog.log("PivotTarget", pivot.getTarget());
        DogLog.log("PivotError", pivot.getError());
        DogLog.log("PivotRatchetPosition", pivot.getRatchetStateValue());
        DogLog.log("PivotRatchetedState", pivot.getRatchetState());
        DogLog.log("PivotIsAtTarget", pivot.isAtTarget());
        DogLog.log("PivotNearTarget", pivot.nearTarget());
    }
}
