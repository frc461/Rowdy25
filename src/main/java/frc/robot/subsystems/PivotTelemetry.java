package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.constants.Constants;

public class PivotTelemetry {
    private final Pivot pivot;

    public PivotTelemetry(Pivot pivot) {
        this.pivot = pivot;
    }

    private final NetworkTable pivotTelemetryTable = Constants.NT_INSTANCE.getTable("PivotTelemetry");
    private final DoublePublisher pivotPosePub = pivotTelemetryTable.getDoubleTopic("Pivot Pose").publish();
    private final DoublePublisher pivotTargetPub = pivotTelemetryTable.getDoubleTopic("Pivot Target Pose").publish();
    private final DoublePublisher pivotErrorPub = pivotTelemetryTable.getDoubleTopic("Pivot Error").publish();
    private final BooleanPublisher pivotIsRatcheted = pivotTelemetryTable.getBooleanTopic("Pivot Is Ratcheted").publish();
    private final BooleanPublisher pivotLowerSwitchTriggered = pivotTelemetryTable.getBooleanTopic("Pivot Lower Switch Triggered").publish();

    public void publishValues() {
        pivotPosePub.set(pivot.getPosition());
        pivotTargetPub.set(pivot.getTarget());
        pivotErrorPub.set(pivot.getError());
        pivotIsRatcheted.set(pivot.isRatcheted());
        pivotLowerSwitchTriggered.set(pivot.lowerSwitchTriggered());

        logValues();
    }

    private void logValues() {
        DogLog.log("PivotPose", pivot.getPosition());
        DogLog.log("PivotTarget", pivot.getTarget());
        DogLog.log("PivotError", pivot.getError());
        DogLog.log("PivotIsRatcheted", pivot.isRatcheted());
        DogLog.log("PivotLowerSwitchTriggered", pivot.lowerSwitchTriggered());
    }
}
