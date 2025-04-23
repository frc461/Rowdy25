package io.github.frc461.rowdy25.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.frc461.rowdy25.RobotStates;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.subsystems.elevator.Elevator;

public class ElevatorCommand extends Command {
    private final Elevator elevator;
    private final DoubleSupplier manualAxisValue;
    private final DoubleSupplier pivotPosition;
    private final RobotStates robotStates;

    public ElevatorCommand(Elevator elevator, DoubleSupplier manualAxisValue, DoubleSupplier pivotPosition, RobotStates robotStates) {
        this.elevator = elevator;
        this.manualAxisValue = manualAxisValue;
        this.pivotPosition = pivotPosition;
        this.robotStates = robotStates;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double axisValue = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND) * 0.25;
        if (axisValue != 0.0) {
            elevator.setManualState();
            robotStates.setManualState();
            elevator.move(axisValue);
        } else {
            elevator.holdTarget(pivotPosition.getAsDouble());
        }
    }
}
