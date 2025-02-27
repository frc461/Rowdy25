package frc.robot.subsystems.climb;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.constants.Constants;

public class ClimbTelemetry {
    private final Climb climb;

    public ClimbTelemetry(Climb climb) {
        this.climb = climb;
    }

    private final NetworkTable climbTelemetryTable = Constants.NT_INSTANCE.getTable("ClimbTelemetry");
    private final StringPublisher climbStatePub = climbTelemetryTable.getStringTopic("Climb State").publish();
    private final DoublePublisher climbPositionPub = climbTelemetryTable.getDoubleTopic("Climb Position").publish();
    private final DoublePublisher climbTargetPub = climbTelemetryTable.getDoubleTopic("Climb Target").publish();

    public void publishValues() {
        climbStatePub.set(climb.getState().name());
        climbPositionPub.set(climb.getPosition());
        climbTargetPub.set(climb.getTarget());

        logValues();
    }

    private void logValues() {
        DogLog.log("ClimbState", climb.getState());
        DogLog.log("ClimbPosition", climb.getPosition());
        DogLog.log("ClimbTarget", climb.getTarget());
    }
}
