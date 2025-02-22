package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.constants.Constants;
import frc.robot.subsystems.wrist.Wrist;

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
        if (axisValue != 0.0 && robotStates.enableManualState) {
            wrist.setManualState();
            robotStates.setManualState();
            wrist.moveWrist(axisValue, pivotPosition.getAsDouble(), elevatorPosition.getAsDouble());
        } else {
            wrist.holdTarget(pivotPosition.getAsDouble());
        }
        wrist.setTarget(pivotPosition.getAsDouble(), elevatorPosition.getAsDouble());
    }
}
