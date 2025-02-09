package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.constants.Constants;

public class ElevatorTelemetry {
    private final Elevator elevator;

    public ElevatorTelemetry(Elevator elevator) {
        this.elevator = elevator;
    }

    private final NetworkTable elevatorTelemetryTable = Constants.NT_INSTANCE.getTable("ElevatorTelemetry");
    private final DoublePublisher elevatorPositionInchesPub = elevatorTelemetryTable.getDoubleTopic("Elevator Position (in)").publish();
    private final DoublePublisher elevatorPositionMetersPub = elevatorTelemetryTable.getDoubleTopic("Elevator Position (m)").publish();
    private final DoublePublisher elevatorTargetPub = elevatorTelemetryTable.getDoubleTopic("Elevator Target").publish();
    private final StringPublisher elevatorStatePub = elevatorTelemetryTable.getStringTopic("Elevator State").publish();
    private final BooleanPublisher elevatorAtTargetPub = elevatorTelemetryTable.getBooleanTopic("Elevator At Target").publish();

    public void publishValues() {
        elevatorPositionInchesPub.set(elevator.getPosition());
        elevatorPositionMetersPub.set(Units.inchesToMeters(elevator.getPosition()));
        elevatorTargetPub.set(elevator.getTarget());
        elevatorStatePub.set(elevator.getState().name());
        elevatorAtTargetPub.set(elevator.isAtTarget());

        logValues();
    }

    private void logValues() {
        DogLog.log("ElevatorPosition", elevator.getPosition());
        DogLog.log("ElevatorTarget", elevator.getTarget());
        DogLog.log("ElevatorState", elevator.getState());
        DogLog.log("ElevatorIsAtTarget", elevator.isAtTarget());
    }
}
