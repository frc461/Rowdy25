package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.wrist.Wrist;

import java.util.function.DoubleSupplier;

public class WristCommand extends Command {
    private final Wrist wrist;
    private final DoubleSupplier controllerValue;

    public WristCommand(Wrist wrist, DoubleSupplier controllerValue) {
        this.wrist = wrist;
        this.controllerValue = controllerValue;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        double val = MathUtil.applyDeadband(controllerValue.getAsDouble(), Constants.DEADBAND);
        if (val != 0.0) {
            wrist.setManualState();
            wrist.moveWrist(val);
        }
        if (wrist.getState() != Wrist.State.MANUAL) {
            wrist.holdTarget();
        }
    }
}
