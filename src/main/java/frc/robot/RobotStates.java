package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.VisionUtil;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

public class RobotStates {
    public enum State { // TODO: IMPLEMENT CLIMBING
        STOW,
        MANUAL,
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
    private final SendableChooser<State> stateChooser = new SendableChooser<>();

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
        Arrays.stream(State.values()).forEach(state -> stateChooser.addOption(state.name(), state));
        stateChooser.onChange(state -> currentState = stateChooser.getSelected()); // TODO SHOP: TEST CHOOSER
        SmartDashboard.putData(stateChooser);
    }

    public void setStowState() {
        currentState = State.STOW;
    }

    public void setManualState() {
        currentState = State.MANUAL;
    }

    public void setOuttakeState() {
        currentState = State.OUTTAKE;
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

    public void toggleNearestAlgaeScoringLocation(boolean net) {
        currentState = net
            ? currentState == State.NET ? State.OUTTAKE : State.NET
            : currentState == State.PROCESSOR ? State.OUTTAKE : State.PROCESSOR;
    }

    public void toggleProcessorState() {
        currentState = currentState == State.PROCESSOR ? State.OUTTAKE : State.PROCESSOR;
    }

    public void toggleNetState() {
        currentState = currentState == State.NET ? State.OUTTAKE : State.NET;
    }

    public void configureToggleStateTriggers(Swerve swerve, Elevator elevator, Intake intake, Pivot pivot, Wrist wrist) {
        
        stowState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setIdleState)
                        .andThen(wrist::setStowState)
                        .andThen(elevator::setStowState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(pivot::setStowState)
        );

        outtakeState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setOuttakeState)
                        .andThen(new WaitUntilCommand(() -> !intake.hasAlgae() && !intake.hasCoral()))
                        .andThen(this::setStowState)
        );

        coralStationState.onTrue(
                new InstantCommand(swerve::setCoralStationHeadingMode)
                        .andThen(intake::setIntakeState)
                        .andThen(pivot::setCoralStationState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setCoralStationState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setCoralStationState)
                        .andThen(new WaitUntilCommand(() -> intake.hasAlgae() || intake.hasCoral()))
                        .andThen(this::setStowState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !coralStationState.getAsBoolean())
        );

        groundCoralState.onTrue(
                new InstantCommand(swerve::setObjectHeadingMode)
                        .andThen(intake::setIntakeState)
                        .andThen(wrist::setGroundCoralState)
                        .andThen(elevator::setGroundCoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(pivot::setGroundCoralState)
                        .andThen(new WaitUntilCommand(VisionUtil.Photon.Color::hasCoralTargets))
                        .andThen(swerve.directMoveToObject().asProxy())
                        .andThen(new WaitUntilCommand(() -> intake.hasAlgae() || intake.hasCoral()))
                        .andThen(this::setStowState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !groundCoralState.getAsBoolean())
        );

        groundAlgaeState.onTrue(
                new InstantCommand(swerve::setObjectHeadingMode)
                        .andThen(intake::setIntakeState)
                        .andThen(wrist::setGroundAlgaeState)
                        .andThen(elevator::setGroundAlgaeState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(pivot::setGroundAlgaeState)
                        .andThen(new WaitUntilCommand(VisionUtil.Photon.Color::hasAlgaeTargets))
                        .andThen(swerve.directMoveToObject().asProxy()) // TODO: MOVE TO ALGAE VS CORAL
                        .andThen(new WaitUntilCommand(() -> intake.hasAlgae() || intake.hasCoral()))
                        .andThen(this::setStowState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !groundAlgaeState.getAsBoolean())
        );

        l1CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingMode)
                        .andThen(transition(elevator, wrist, pivot::nearTarget))
                        .andThen(pivot::setL1CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL1CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL1CoralState)
                        .until(() -> !l1CoralState.getAsBoolean())
        );

        l2CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingMode)
                        .andThen(transition(elevator, wrist, pivot::nearTarget))
                        .andThen(pivot::setL2CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL2CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL2CoralState)
                        .until(() -> !l2CoralState.getAsBoolean())
        );

        l3CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingMode)
                        .andThen(transition(elevator, wrist, pivot::nearTarget))
                        .andThen(pivot::setL3CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL3CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL3CoralState)
                        .until(() -> !l3CoralState.getAsBoolean())
        );

        l4CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingMode)
                        .andThen(transition(elevator, wrist, pivot::nearTarget))
                        .andThen(pivot::setL4CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL4CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL4CoralState)
                        .until(() -> !l4CoralState.getAsBoolean())
        );

        lowReefAlgaeState.onTrue(
                new InstantCommand(swerve::setReefTagHeadingMode)
                        .andThen(transition(elevator, wrist, pivot::nearTarget))
                        .andThen(intake::setIntakeState)
                        .andThen(pivot::setLowReefAlgaeState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setLowReefAlgaeState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setLowReefAlgaeState)
                        .andThen(new WaitUntilCommand(() -> intake.hasAlgae() || intake.hasCoral()))
                        .andThen(this::setStowState)
                        .until(swerve.localizer::nearestAlgaeIsHigh)
                        .andThen(this::toggleHighReefAlgaeState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !lowReefAlgaeState.getAsBoolean())
        );

        highReefAlgaeState.onTrue(
                new InstantCommand(swerve::setReefTagHeadingMode)
                        .andThen(transition(elevator, wrist, pivot::nearTarget))
                        .andThen(intake::setIntakeState)
                        .andThen(pivot::setHighReefAlgaeState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setHighReefAlgaeState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setHighReefAlgaeState)
                        .andThen(new WaitUntilCommand(() -> intake.hasAlgae() || intake.hasCoral()))
                        .andThen(this::setStowState)
                        .until(() -> !swerve.localizer.nearestAlgaeIsHigh())
                        .andThen(this::toggleLowReefAlgaeState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !highReefAlgaeState.getAsBoolean())
        );

        processorState.onTrue(
                new InstantCommand(swerve::setAlgaeScoringHeadingMode)
                        .andThen(wrist::setProcessorState)
                        .andThen(elevator::setProcessorState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(pivot::setProcessorState)
                        .until(() -> !processorState.getAsBoolean())
        );

        netState.onTrue(
                new InstantCommand(swerve::setAlgaeScoringHeadingMode)
                        .andThen(transition(elevator, wrist, pivot::nearTarget))
                        .andThen(pivot::setNetState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setNetState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setNetState)
                        .until(() -> !netState.getAsBoolean())
        );
    }

    private Command transition(Elevator elevator, Wrist wrist, BooleanSupplier pivotNearTarget) { // TODO SHOP: TEST SMOOTHER TRANSITIONS
        return new ConditionalCommand(
                new InstantCommand(wrist::setStowState).andThen(elevator::setStowState).andThen(new WaitUntilCommand(elevator::nearTarget)),
                new InstantCommand(wrist::setStowState),
                pivotNearTarget
        );
    }

    public void publishValues() {
        robotStatesPub.set(currentState.name());
    }
}
