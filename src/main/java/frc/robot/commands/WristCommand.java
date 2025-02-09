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

    public WristCommand(Wrist wrist, DoubleSupplier manualAxisValue, DoubleSupplier pivotPosition) {
        this.wrist = wrist;
        this.manualAxisValue = manualAxisValue;
        this.pivotPosition = pivotPosition;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        double val = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND);
        if (val != 0.0) {
            wrist.setManualState();
            wrist.moveWrist(val);
        } else {
            wrist.holdTarget(pivotPosition.getAsDouble());
        }
    }
}
