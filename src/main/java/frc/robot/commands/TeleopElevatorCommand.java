package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.constants.Constants;

public class TeleopElevatorCommand extends Command {
    private final Elevator elevator;
    private final DoubleSupplier motionSup;

    public TeleopElevatorCommand(Elevator elevator, DoubleSupplier motionSup) {
        this.elevator = elevator;
        this.motionSup = motionSup;
        addRequirements(this.elevator);
    }

    @Override
    public void execute() {
        /* Apply Deadband */
        double axisValue = MathUtil.applyDeadband(motionSup.getAsDouble(), Constants.DEADBAND);

        /* Move Elevator */
        if (axisValue != 0) {
            elevator.moveElevator(motionSup.getAsDouble());
        } else {
            elevator.holdTarget();
        }
    } 
}
