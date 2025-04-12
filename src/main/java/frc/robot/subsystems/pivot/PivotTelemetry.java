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
    private final StringPublisher pivotStatePub = pivotTelemetryTable.getStringTopic("Pivot State").publish();
    private final DoublePublisher pivotGravityGainsPub = pivotTelemetryTable.getDoubleTopic("Pivot Gravity Gains").publish();
    private final DoublePublisher pivotUpRatchetPositionPub = pivotTelemetryTable.getDoubleTopic("Pivot Up Ratchet Position").publish();
    private final StringPublisher pivotIsUpRatcheted = pivotTelemetryTable.getStringTopic("Pivot Up Ratchet State").publish();
    private final DoublePublisher pivotDownRatchetPositionPub = pivotTelemetryTable.getDoubleTopic("Pivot Down Ratchet Position").publish();
    private final StringPublisher pivotIsDownRatcheted = pivotTelemetryTable.getStringTopic("Pivot Down Ratchet State").publish();
    private final BooleanPublisher pivotAtTargetPub = pivotTelemetryTable.getBooleanTopic("Pivot At Target").publish();
    private final BooleanPublisher pivotNearTargetPub = pivotTelemetryTable.getBooleanTopic("Pivot Near Target").publish();
    private final DoublePublisher pivotCurrent = pivotTelemetryTable.getDoubleTopic("Pivot Current").publish();

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
