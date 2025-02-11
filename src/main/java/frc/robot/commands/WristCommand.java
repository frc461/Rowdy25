package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.wrist.Wrist;

import java.util.function.DoubleSupplier;

public class WristCommand extends Command {
    private final Wrist wrist;
    private final DoubleSupplier manualAxisValue;
    private final DoubleSupplier pivotPosition;
    private final DoubleSupplier elevatorPosition;

    public WristCommand(Wrist wrist, DoubleSupplier manualAxisValue, DoubleSupplier pivotPosition, DoubleSupplier elevatorPosition) {
        this.wrist = wrist;
        this.manualAxisValue = manualAxisValue;
        this.pivotPosition = pivotPosition;
        this.elevatorPosition = elevatorPosition;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        double axisValue = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND) * 0.1;
        if (axisValue != 0.0) {
            wrist.setManualState();
            wrist.moveWrist(axisValue, pivotPosition.getAsDouble(), elevatorPosition.getAsDouble());
        } else {
            wrist.holdTarget(pivotPosition.getAsDouble());
        }
        wrist.setTarget(pivotPosition.getAsDouble(), elevatorPosition.getAsDouble());
    }
}
