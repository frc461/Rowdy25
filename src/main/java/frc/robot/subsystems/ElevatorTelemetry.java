package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.constants.Constants;

public class ElevatorTelemetry {
    private final Elevator elevator;

    public ElevatorTelemetry(Elevator elevator) {
        this.elevator = elevator;
    }

    private final NetworkTable elevatorTelemetryTable = Constants.NT_INSTANCE.getTable("ElevatorTelemetry");
    private final DoublePublisher elevatorPosePub = elevatorTelemetryTable.getDoubleTopic("Elevator Pose").publish();
    private final DoublePublisher elevatorTargetPub = elevatorTelemetryTable.getDoubleTopic("Elevator Target").publish();

    public void publishValues() {
        elevatorPosePub.set(elevator.getPosition());
        elevatorTargetPub.set(elevator.getTarget());

        logValues();
    }

    private void logValues() {
        DogLog.log("ElevatorPose", elevator.getPosition());
        DogLog.log("ElevatorTarget", elevator.getTarget());
    }
}
