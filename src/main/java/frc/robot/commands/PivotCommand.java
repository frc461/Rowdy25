package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.GravityGainsCalculator;

import java.util.function.DoubleSupplier;

public class PivotCommand extends Command {
    private final Pivot pivot;
    private final DoubleSupplier manualAxisValue;
    private final DoubleSupplier elevatorPosition;
    private final DoubleSupplier wristPosition;

    private final GravityGainsCalculator gravityGainsCalculator = new GravityGainsCalculator(
            Constants.PivotConstants.AXIS_POSITION,
            Constants.WristConstants.AXIS_POSITION,
            Constants.WristConstants.AXIS_TO_ZERO_COM,
            Constants.ElevatorConstants.ZERO_UPRIGHT_COM,
            Constants.ElevatorConstants.COM_TO_STAGE_2_RATIO,
            Constants.ElevatorConstants.STAGE_2_LIMIT,
            Constants.ElevatorConstants.COM_TO_STAGE_3_RATIO,
            Constants.ElevatorConstants.MASS_LBS,
            Constants.WristConstants.MASS_LBS,
            Constants.PivotConstants.G
    );

    public PivotCommand(Pivot pivot, DoubleSupplier manualAxisValue, DoubleSupplier elevatorPosition, DoubleSupplier wristPosition) {
        this.pivot = pivot;
        this.manualAxisValue = manualAxisValue;
        this.elevatorPosition = elevatorPosition;
        this.wristPosition = wristPosition;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        double axisValue = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND) * 0.1;
        if (axisValue != 0.0) {
            pivot.setManualState();
            pivot.movePivot(axisValue);
        } else {
            pivot.holdTarget(gravityGainsCalculator.calculateGFromPositions(pivot.getPosition(), wristPosition.getAsDouble(), elevatorPosition.getAsDouble()));
        }
    }
}
