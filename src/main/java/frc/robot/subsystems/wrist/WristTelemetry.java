package frc.robot.subsystems.wrist;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.constants.Constants;

public class WristTelemetry {
    private final Wrist wrist;

    public WristTelemetry(Wrist wrist) {
        this.wrist = wrist;
    }

    private final NetworkTable wristTelemetryTable = Constants.NT_INSTANCE.getTable("WristTelemetry");
    private final DoublePublisher wristPosePub = wristTelemetryTable.getDoubleTopic("Wrist Pose").publish();
    private final DoublePublisher wristTargetPub = wristTelemetryTable.getDoubleTopic("Wrist Target").publish();
    private final DoublePublisher wristErrorPub = wristTelemetryTable.getDoubleTopic("Wrist Error").publish();

    public void publishValues() {
        wristPosePub.set(wrist.getPosition());
        wristTargetPub.set(wrist.getTarget());
        wristErrorPub.set(wrist.getError());

        logValues();
    }

    private void logValues() {
        //DogLog.log("WristPose", wrist.getPosition());
        //DogLog.log("WristTarget", wrist.getTarget());
        //DogLog.log("WristError", wrist.getError());
        //DogLog.log("WristLowerSwitchTriggered", wrist.lowerSwitchTriggered());
    }
}
