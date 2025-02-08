package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommand extends Command {
    private final Elevator elevator;
    private final DoubleSupplier controllerValue;

    public ElevatorCommand(Elevator elevator, DoubleSupplier controllerValue) {
        this.elevator = elevator;
        this.controllerValue = controllerValue;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double val = MathUtil.applyDeadband(controllerValue.getAsDouble(), Constants.DEADBAND);
        if (val != 0.0) {
            elevator.setManualState();
            elevator.moveElevator(val);
        }
        if (elevator.getState() != Elevator.State.MANUAL) {
            elevator.holdTarget();
        }
    }
}
