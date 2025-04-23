package io.github.frc461.rowdy25.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.frc461.rowdy25.RobotStates;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.subsystems.wrist.Wrist;

import java.util.function.DoubleSupplier;

public class WristCommand extends Command {
    private final Wrist wrist;
    private final DoubleSupplier manualAxisValue;
    private final DoubleSupplier pivotPosition;
    private final DoubleSupplier elevatorPosition;
    private final RobotStates robotStates;

    public WristCommand(Wrist wrist, DoubleSupplier manualAxisValue, DoubleSupplier pivotPosition, DoubleSupplier elevatorPosition, RobotStates robotStates) {
        this.wrist = wrist;
        this.manualAxisValue = manualAxisValue;
        this.pivotPosition = pivotPosition;
        this.elevatorPosition = elevatorPosition;
        this.robotStates = robotStates;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        double axisValue = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND) * 0.25;
        if (axisValue != 0.0) {
            wrist.setManualState();
            robotStates.setManualState();
            wrist.move(axisValue, pivotPosition.getAsDouble(), elevatorPosition.getAsDouble());
        } else {
            wrist.holdTarget(pivotPosition.getAsDouble());
        }
        wrist.setTarget(pivotPosition.getAsDouble(), elevatorPosition.getAsDouble());
    }
}
