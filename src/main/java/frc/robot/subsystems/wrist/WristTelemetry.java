package frc.robot.subsystems.wrist;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.constants.Constants;

public class WristTelemetry {
    private Wrist wrist;

    public WristTelemetry(Wrist wrist) {
        this.wrist = wrist;
    }

    private NetworkTable wristTelemetryTable = Constants.NT_INSTANCE.getTable("WristTelemetry");
    private DoublePublisher wristPosePub = wristTelemetryTable.getDoubleTopic("Wrist Pose").publish();
    private DoublePublisher wristTargetPub = wristTelemetryTable.getDoubleTopic("Wrist Target").publish();
    private DoublePublisher wristErrorPub = wristTelemetryTable.getDoubleTopic("Wrist Error").publish();
    private BooleanPublisher wristLowerSwitchTriggered = wristTelemetryTable.getBooleanTopic("Wrist Switch Triggered").publish();

    public void publishValues() {
        wristPosePub.set(wrist.getPosition());
        wristTargetPub.set(wrist.getTarget());
        wristErrorPub.set(wrist.getError());
        wristLowerSwitchTriggered.set(wrist.lowerSwitchTriggered());

        logValues();
    }

    private void logValues() {
        DogLog.log("WristPose", wrist.getPosition());
        DogLog.log("WristTarget", wrist.getTarget());
        DogLog.log("WristError", wrist.getError());
        DogLog.log("WristLowerSwitchTriggered", wrist.lowerSwitchTriggered());
    }
}
