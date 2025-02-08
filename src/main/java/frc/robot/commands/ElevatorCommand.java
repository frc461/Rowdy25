package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommand extends Command {
    private final Elevator elevator;

    public ElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.holdTarget();
    }
}
