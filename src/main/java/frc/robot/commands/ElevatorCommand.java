package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommand extends Command {
    private final Elevator elevator;
    private final DoubleSupplier manualAxisValue;
    private final DoubleSupplier pivotPosition;

    public ElevatorCommand(Elevator elevator, DoubleSupplier manualAxisValue, DoubleSupplier pivotPosition) {
        this.elevator = elevator;
        this.manualAxisValue = manualAxisValue;
        this.pivotPosition = pivotPosition;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double axisValue = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND) * 0.1;
        if (axisValue != 0.0) {
            elevator.setManualState();
            elevator.moveElevator(axisValue);
        } else {
            elevator.holdTarget(pivotPosition.getAsDouble());
        }
    }
}
