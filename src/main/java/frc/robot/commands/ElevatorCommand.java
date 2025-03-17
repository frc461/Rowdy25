package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;

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
