package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.VisionUtil;

public class RobotStates {
    public enum State { // TODO: IMPLEMENT CLIMBING
        STOW,
        OUTTAKE,
        CORAL_STATION,
        GROUND_CORAL,
        GROUND_ALGAE,
        L1_CORAL,
        L2_CORAL,
        L3_CORAL,
        L4_CORAL,
        LOW_REEF_ALGAE,
        HIGH_REEF_ALGAE,
        PROCESSOR,
        NET
    }

    private State currentState;

    public final Trigger stowState = new Trigger(() -> currentState == State.STOW);
    public final Trigger outtakeState = new Trigger(() -> currentState == State.OUTTAKE);
    public final Trigger coralStationState = new Trigger(() -> currentState == State.CORAL_STATION);
    public final Trigger groundCoralState = new Trigger(() -> currentState == State.GROUND_CORAL);
    public final Trigger groundAlgaeState = new Trigger(() -> currentState == State.GROUND_ALGAE);
    public final Trigger l1CoralState = new Trigger(() -> currentState == State.L1_CORAL);
    public final Trigger l2CoralState = new Trigger(() -> currentState == State.L2_CORAL);
    public final Trigger l3CoralState = new Trigger(() -> currentState == State.L3_CORAL);
    public final Trigger l4CoralState = new Trigger(() -> currentState == State.L4_CORAL);
    public final Trigger lowReefAlgaeState = new Trigger(() -> currentState == State.LOW_REEF_ALGAE);
    public final Trigger highReefAlgaeState = new Trigger(() -> currentState == State.HIGH_REEF_ALGAE);
    public final Trigger processorState = new Trigger(() -> currentState == State.PROCESSOR);
    public final Trigger netState = new Trigger(() -> currentState == State.NET);

    private final NetworkTable robotStatesTelemetryTable = Constants.NT_INSTANCE.getTable("RobotStates");
    private final StringPublisher robotStatesPub = robotStatesTelemetryTable.getStringTopic("Current Robot State").publish();

    public RobotStates() {
        currentState = State.STOW;
    }

    public void setStowState() {
        currentState = State.STOW;
    }

    public void toggleCoralStationState() {
        currentState = currentState == State.CORAL_STATION ? State.STOW : State.CORAL_STATION;
    }

    public void toggleGroundCoralState() {
        currentState = currentState == State.GROUND_CORAL ? State.STOW : State.GROUND_CORAL;
    }

    public void toggleGroundAlgaeState() {
        currentState = currentState == State.GROUND_ALGAE ? State.STOW : State.GROUND_ALGAE;
    }

    public void toggleL1CoralState() {
        currentState = currentState == State.L1_CORAL ? State.OUTTAKE : State.L1_CORAL;
    }

    public void toggleL2CoralState() {
        currentState = currentState == State.L2_CORAL ? State.OUTTAKE : State.L2_CORAL;
    }

    public void toggleL3CoralState() {
        currentState = currentState == State.L3_CORAL ? State.OUTTAKE : State.L3_CORAL;
    }

    public void toggleL4CoralState() {
        currentState = currentState == State.L4_CORAL ? State.OUTTAKE : State.L4_CORAL;
    }

    public void toggleLowReefAlgaeState() {
        currentState = currentState == State.LOW_REEF_ALGAE ? State.STOW : State.LOW_REEF_ALGAE;
    }

    public void toggleHighReefAlgaeState() {
        currentState = currentState == State.HIGH_REEF_ALGAE ? State.STOW : State.HIGH_REEF_ALGAE;
    }

    public void toggleNearestReefAlgaeState(boolean high) {
        currentState = high
                ? currentState == State.HIGH_REEF_ALGAE ? State.STOW : State.HIGH_REEF_ALGAE
                : currentState == State.LOW_REEF_ALGAE ? State.STOW : State.LOW_REEF_ALGAE;
    }

    public void toggleProcessorState() {
        currentState = currentState == State.PROCESSOR ? State.OUTTAKE : State.PROCESSOR;
    }

    public void toggleNetState() {
        currentState = currentState == State.NET ? State.OUTTAKE : State.NET;
    }

    public void configureToggleStateTriggers(Swerve swerve, Elevator elevator, Intake intake, Pivot pivot, Wrist wrist) { // TODO SHOP: TEST THIS
        stowState.onTrue(
                new InstantCommand(intake::setIdleState)
                        .andThen(wrist::setStowState)
                        .andThen(new WaitUntilCommand(wrist::nearTarget))
                        .andThen(elevator::setStowState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(pivot::setStowState)
        );

        outtakeState.onTrue(
                new InstantCommand(intake::setOuttakeState)
                        .andThen(new WaitUntilCommand(intake::atIdleState))
                        .andThen(this::setStowState)
        );

        coralStationState.onTrue(
                new InstantCommand(intake::setIntakeState)
                        .andThen(wrist::setCoralStationState)
                        .andThen(new WaitUntilCommand(wrist::nearTarget))
                        .andThen(elevator::setCoralStationState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(pivot::setCoralStationState)
                        .andThen(new WaitUntilCommand(() -> intake.atIdleState() || intake.atHasAlgaeState()))
                        .andThen(this::setStowState)
                        .until(() -> !coralStationState.getAsBoolean())
        );

        groundCoralState.onTrue(
                new InstantCommand(intake::setIntakeState)
                        .andThen(wrist::setGroundCoralState)
                        .andThen(new WaitUntilCommand(wrist::nearTarget))
                        .andThen(elevator::setGroundCoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(pivot::setGroundCoralState)
                        .andThen(new WaitUntilCommand(VisionUtil.Photon.Color::hasCoralTargets))
                        .andThen(swerve.directMoveToObject().asProxy())
                        .andThen(new WaitUntilCommand(() -> intake.atIdleState() || intake.atHasAlgaeState()))
                        .andThen(this::setStowState)
                        .until(() -> !groundCoralState.getAsBoolean())
        );

        groundAlgaeState.onTrue(
                new InstantCommand(intake::setIntakeState)
                        .andThen(wrist::setGroundAlgaeState)
                        .andThen(new WaitUntilCommand(wrist::nearTarget))
                        .andThen(elevator::setGroundAlgaeState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(pivot::setGroundAlgaeState)
                        .andThen(new WaitUntilCommand(VisionUtil.Photon.Color::hasAlgaeTargets))
                        .andThen(swerve.directMoveToObject().asProxy()) // TODO: MOVE TO ALGAE VS CORAL
                        .andThen(new WaitUntilCommand(() -> intake.atIdleState() || intake.atHasAlgaeState()))
                        .andThen(this::setStowState)
                        .until(() -> !groundAlgaeState.getAsBoolean())
        );

        l1CoralState.onTrue(
                resetState(elevator, intake)
                        .andThen(new WaitUntilCommand(() -> elevator.nearTarget() && pivot.nearTarget() && wrist.nearTarget()))
                        .andThen(pivot::setL1CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL1CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL1CoralState)
                        .andThen(new WaitUntilCommand(() -> !intake.hasCoral() && !intake.hasAlgae()))
                        .andThen(this::setStowState)
                        .until(() -> !l1CoralState.getAsBoolean())
        );

        l2CoralState.onTrue(
                resetState(elevator, intake)
                        .andThen(new WaitUntilCommand(() -> elevator.nearTarget() && pivot.nearTarget() && wrist.nearTarget()))
                        .andThen(pivot::setL2CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL2CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL2CoralState)
                        .andThen(new WaitUntilCommand(() -> !intake.hasCoral() && !intake.hasAlgae()))
                        .andThen(this::setStowState)
                        .until(() -> !l2CoralState.getAsBoolean())
        );

        l3CoralState.onTrue(
                resetState(elevator, intake)
                        .andThen(new WaitUntilCommand(() -> elevator.nearTarget() && pivot.nearTarget() && wrist.nearTarget()))
                        .andThen(pivot::setL3CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL3CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL3CoralState)
                        .andThen(new WaitUntilCommand(() -> !intake.hasCoral() && !intake.hasAlgae()))
                        .andThen(this::setStowState)
                        .until(() -> !l3CoralState.getAsBoolean())
        );

        l4CoralState.onTrue(
                resetState(elevator, intake)
                        .andThen(new WaitUntilCommand(() -> elevator.nearTarget() && pivot.nearTarget() && wrist.nearTarget()))
                        .andThen(pivot::setL4CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL4CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL4CoralState)
                        .andThen(new WaitUntilCommand(() -> !intake.hasCoral() && !intake.hasAlgae()))
                        .andThen(this::setStowState)
                        .until(() -> !l4CoralState.getAsBoolean())
        );

        lowReefAlgaeState.onTrue(
                resetState(elevator, intake)
                        .andThen(new WaitUntilCommand(() -> elevator.nearTarget() && pivot.nearTarget() && wrist.nearTarget()))
                        .andThen(intake::setIntakeState)
                        .andThen(pivot::setLowReefAlgaeState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setLowReefAlgaeState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setLowReefAlgaeState)
                        .andThen(new WaitUntilCommand(() -> intake.atIdleState() || intake.atHasAlgaeState()))
                        .andThen(this::setStowState)
                        .until(() -> elevator.nearestAlgaeIsHigh(swerve.localizer.getStrategyPose()))
                        .andThen(this::toggleHighReefAlgaeState)
                        .until(() -> !lowReefAlgaeState.getAsBoolean())
        );

        highReefAlgaeState.onTrue(
                resetState(elevator, intake)
                        .andThen(new WaitUntilCommand(() -> elevator.nearTarget() && pivot.nearTarget() && wrist.nearTarget()))
                        .andThen(intake::setIntakeState)
                        .andThen(pivot::setHighReefAlgaeState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setHighReefAlgaeState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setHighReefAlgaeState)
                        .andThen(new WaitUntilCommand(() -> intake.atIdleState() || intake.atHasAlgaeState()))
                        .andThen(this::setStowState)
                        .until(() -> !elevator.nearestAlgaeIsHigh(swerve.localizer.getStrategyPose()))
                        .andThen(this::toggleLowReefAlgaeState)
                        .until(() -> !highReefAlgaeState.getAsBoolean())
        );

        processorState.onTrue(
                new InstantCommand(wrist::setProcessorState)
                        .andThen(new WaitUntilCommand(wrist::nearTarget))
                        .andThen(elevator::setProcessorState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(pivot::setProcessorState)
                        .andThen(new WaitUntilCommand(() -> !intake.hasCoral() && !intake.hasAlgae()))
                        .andThen(this::setStowState)
                        .until(() -> !processorState.getAsBoolean())
        );

        netState.onTrue(
                resetState(elevator, intake)
                        .andThen(new WaitUntilCommand(() -> elevator.nearTarget() && pivot.nearTarget() && wrist.nearTarget()))
                        .andThen(pivot::setNetState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setNetState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setNetState)
                        .andThen(new WaitUntilCommand(() -> !intake.hasCoral() && !intake.hasAlgae()))
                        .andThen(this::setStowState)
                        .until(() -> !netState.getAsBoolean())
        );
    }

    private Command resetState(Elevator elevator, Intake intake) {
        return new InstantCommand(intake::setIdleState)
                        .andThen(elevator::setStowState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget));
    }

    public void publishValues() {
        robotStatesPub.set(currentState.name());
    }
}
