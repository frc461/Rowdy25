package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.pivot.Pivot;

import java.util.function.DoubleSupplier;

public class PivotCommand extends Command {
    private final Pivot pivot;
    private final DoubleSupplier manualAxisValue;

    public PivotCommand(Pivot pivot, DoubleSupplier manualAxisValue) {
        this.pivot = pivot;
        this.manualAxisValue = manualAxisValue;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        double axisValue = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND) * 0.3;
        if (axisValue != 0.0) {
            pivot.setManualState();
            pivot.movePivot(axisValue);
        } else {
            pivot.holdTarget();
        }
    }
}
