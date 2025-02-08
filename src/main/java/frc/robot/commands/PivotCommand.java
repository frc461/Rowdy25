package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.pivot.Pivot;

import java.util.function.DoubleSupplier;

public class PivotCommand extends Command {
    private final Pivot pivot;
    private final DoubleSupplier controllerValue;

    public PivotCommand(Pivot pivot, DoubleSupplier controllerValue) {
        this.pivot = pivot;
        this.controllerValue = controllerValue;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        double val = MathUtil.applyDeadband(controllerValue.getAsDouble(), Constants.DEADBAND);
        if (val != 0.0) {
            pivot.setManualState();
            pivot.movePivot(val);
        }
        if (pivot.getState() != Pivot.State.MANUAL) {
            pivot.holdTarget();
        }
    }
}
