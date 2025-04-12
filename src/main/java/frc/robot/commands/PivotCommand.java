package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.constants.Constants;
import frc.robot.subsystems.pivot.Pivot;

import java.util.function.DoubleSupplier;

public class PivotCommand extends Command {
    private final Pivot pivot;
    private final DoubleSupplier manualAxisValue;
    private final DoubleSupplier elevatorPosition;
    private final DoubleSupplier wristPosition;
    private final RobotStates robotStates;

    public PivotCommand(Pivot pivot, DoubleSupplier manualAxisValue, DoubleSupplier elevatorPosition, DoubleSupplier wristPosition, RobotStates robotStates) {
        this.pivot = pivot;
        this.manualAxisValue = manualAxisValue;
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        this.robotStates = robotStates;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        double axisValue = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND) * 0.3;
        if (axisValue != 0.0) {
            pivot.setManualState();
            robotStates.setManualState();
            pivot.move(axisValue);
        } else {
            pivot.holdTarget(elevatorPosition.getAsDouble(), wristPosition.getAsDouble());
        }
    }
}
