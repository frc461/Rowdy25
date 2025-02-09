package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.constants.Constants;

public class ElevatorTelemetry {
    private final Elevator elevator;

    public ElevatorTelemetry(Elevator elevator) {
        this.elevator = elevator;
    }

    private final NetworkTable elevatorTelemetryTable = Constants.NT_INSTANCE.getTable("ElevatorTelemetry");
    private final DoublePublisher elevatorPositionInchesPub = elevatorTelemetryTable.getDoubleTopic("Elevator Position (m)").publish();
    private final DoublePublisher elevatorPositionMetersPub = elevatorTelemetryTable.getDoubleTopic("Elevator Position (in)").publish();
    private final DoublePublisher elevatorTargetPub = elevatorTelemetryTable.getDoubleTopic("Elevator Target").publish();

    public void publishValues() {
        elevatorPositionInchesPub.set(elevator.getPosition());
        elevatorPositionMetersPub.set(Units.inchesToMeters(elevator.getPosition()));
        elevatorTargetPub.set(elevator.getTarget());

        logValues();
    }

    private void logValues() {
        //DogLog.log("ElevatorPose", elevator.getPosition());
        //DogLog.log("ElevatorTarget", elevator.getTarget());
    }
}
