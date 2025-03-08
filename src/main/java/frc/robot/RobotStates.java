package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.WristCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.DoubleTrueTrigger;
import frc.robot.util.FieldUtil;
import frc.robot.util.vision.PhotonUtil;

import java.util.Arrays;

import dev.doglog.DogLog;

public class RobotStates {
    public enum State {
        STOW,
        MANUAL,
        OUTTAKE,
        INTAKE_OUT,
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
        NET,
        PREPARE_CLIMB,
        CLIMB
    }

    public final Swerve swerve = new Swerve();
    public final Climb climb = new Climb();
    public final Elevator elevator = new Elevator();
    public final Intake intake = new Intake();
    public final Pivot pivot = new Pivot();
    public final Wrist wrist = new Wrist();

    private State currentState = State.STOW;
    private FieldUtil.Reef.Level currentAutoLevel = FieldUtil.Reef.Level.L2;
    private boolean isAutoLevelToggledOn = false;
    private boolean isAutoScoreToggledOn = false;
    private final SendableChooser<State> stateChooser = new SendableChooser<>();

    public final Trigger stowState = new Trigger(() -> currentState == State.STOW);
    public final Trigger outtakeState = new Trigger(() -> currentState == State.OUTTAKE);
    public final Trigger intakeOutState = new Trigger(() -> currentState == State.INTAKE_OUT);
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
    public final Trigger prepareClimbState = new Trigger(() -> currentState == State.PREPARE_CLIMB);
    public final Trigger climbState = new Trigger(() -> currentState == State.CLIMB);

    public final Trigger autoLevelState = new Trigger(() -> isAutoLevelToggledOn);

    public final Trigger atState = new Trigger(() -> elevator.isAtTarget() && pivot.isAtTarget() && wrist.isAtTarget());

    public final Trigger atStowState = new Trigger(() -> wrist.isAtState(Wrist.State.STOW) && elevator.isAtState(Elevator.State.STOW) && pivot.isAtState(Pivot.State.STOW));
    public final Trigger atCoralStationState = new Trigger(() -> wrist.isAtState(Wrist.State.CORAL_STATION) && elevator.isAtState(Elevator.State.CORAL_STATION) && pivot.isAtState(Pivot.State.CORAL_STATION));
    public final Trigger atGroundCoralState = new Trigger(() -> wrist.isAtState(Wrist.State.GROUND_CORAL) && elevator.isAtState(Elevator.State.GROUND_CORAL) && pivot.isAtState(Pivot.State.GROUND_CORAL));
    public final Trigger atGroundAlgaeState = new Trigger(() -> wrist.isAtState(Wrist.State.GROUND_ALGAE) && elevator.isAtState(Elevator.State.GROUND_ALGAE) && pivot.isAtState(Pivot.State.GROUND_ALGAE));
    public final Trigger atL1CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L1_CORAL) && elevator.isAtState(Elevator.State.L1_CORAL) && pivot.isAtState(Pivot.State.L1_CORAL));
    public final Trigger atL2CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L2_CORAL) && elevator.isAtState(Elevator.State.L2_CORAL) && pivot.isAtState(Pivot.State.L2_CORAL));
    public final Trigger atL3CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L3_CORAL) && elevator.isAtState(Elevator.State.L3_CORAL) && pivot.isAtState(Pivot.State.L3_CORAL));
    public final Trigger atL4CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L4_CORAL) && elevator.isAtState(Elevator.State.L4_CORAL) && pivot.isAtState(Pivot.State.L4_CORAL));
    public final Trigger atLowReefAlgaeState = new Trigger(() -> wrist.isAtState(Wrist.State.LOW_REEF_ALGAE) && elevator.isAtState(Elevator.State.LOW_REEF_ALGAE) && pivot.isAtState(Pivot.State.LOW_REEF_ALGAE));
    public final Trigger atHighReefAlgaeState= new Trigger(() -> wrist.isAtState(Wrist.State.HIGH_REEF_ALGAE) && elevator.isAtState(Elevator.State.HIGH_REEF_ALGAE) && pivot.isAtState(Pivot.State.HIGH_REEF_ALGAE));
    public final Trigger atProcessorState = new Trigger(() -> wrist.isAtState(Wrist.State.PROCESSOR) && elevator.isAtState(Elevator.State.PROCESSOR) && pivot.isAtState(Pivot.State.PROCESSOR));
    public final Trigger atNetState = new Trigger(() -> wrist.isAtState(Wrist.State.NET) && elevator.isAtState(Elevator.State.NET) && pivot.isAtState(Pivot.State.NET));
    public final Trigger atClimbState = new Trigger(() -> wrist.isAtState(Wrist.State.CLIMB) && elevator.isAtState(Elevator.State.CLIMB) && pivot.isAtState(Pivot.State.CLIMB));

    public final Trigger atAutoScoreState = switch (currentAutoLevel) {
        case L1 -> atL1CoralState;
        case L2 -> atL2CoralState;
        case L3 -> atL3CoralState;
        case L4 -> atL4CoralState;
    };

    private final NetworkTable robotStatesTelemetryTable = Constants.NT_INSTANCE.getTable("RobotStates");
    private final StringPublisher robotStatesPub = robotStatesTelemetryTable.getStringTopic("Current Robot State").publish();

    public RobotStates() {
        Lights.configureLights();

        Arrays.stream(State.values()).forEach(state -> stateChooser.addOption(state.name(), state));
        stateChooser.onChange(state -> currentState = stateChooser.getSelected());
        SmartDashboard.putData("Robot State Chooser", stateChooser);
    }

    public State getAutoLevelState() {
        return switch (currentAutoLevel) {
            case L1 -> State.L1_CORAL;
            case L2 -> State.L2_CORAL;
            case L3 -> State.L3_CORAL;
            case L4 -> State.L4_CORAL;
        };
    }

    public void setCurrentAutoLevel(FieldUtil.Reef.Level level) {
        currentAutoLevel = level;
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

    public void setIntakeOutState() {
        currentState = State.INTAKE_OUT;
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

    public void toggleL1CoralState(boolean autoScore) {
        currentState = currentState == State.L1_CORAL ? State.OUTTAKE : State.L1_CORAL;
        isAutoScoreToggledOn = currentState == State.L1_CORAL && autoScore;
    }

    public void toggleL1CoralState() {
        toggleL1CoralState(false);
    }

    public void toggleL2CoralState(boolean autoScore) {
        currentState = currentState == State.L2_CORAL ? State.INTAKE_OUT : State.L2_CORAL;
        isAutoScoreToggledOn = currentState == State.L2_CORAL && autoScore;
    }

    public void toggleL2CoralState() {
        toggleL2CoralState(false);
    }

    public void toggleL3CoralState(boolean autoScore) {
        currentState = currentState == State.L3_CORAL ? State.INTAKE_OUT : State.L3_CORAL;
        isAutoScoreToggledOn = currentState == State.L3_CORAL && autoScore;
    }

    public void toggleL3CoralState() {
        toggleL3CoralState(false);
    }

    public void toggleL4CoralState(boolean autoScore) {
        currentState = currentState == State.L4_CORAL ? State.INTAKE_OUT : State.L4_CORAL;
        isAutoScoreToggledOn = currentState == State.L4_CORAL && autoScore;
    }

    public void toggleL4CoralState() {
        toggleL4CoralState(false);
    }

    public void toggleAutoLevelCoralState() {
        isAutoLevelToggledOn = !isAutoLevelToggledOn;
        switch (currentAutoLevel) {
            case L1 -> toggleL1CoralState();
            case L2 -> toggleL2CoralState();
            case L3 -> toggleL3CoralState();
            case L4 -> toggleL4CoralState();
        }
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

    public void toggleProcessorState(boolean autoScore) {
        currentState = currentState == State.PROCESSOR ? State.OUTTAKE : State.PROCESSOR;
        isAutoScoreToggledOn = currentState == State.PROCESSOR && autoScore;
    }

    public void toggleProcessorState() {
        toggleProcessorState(false);
    }

    public void toggleNetState(boolean autoScore) {
        currentState = currentState == State.NET ? State.OUTTAKE : State.NET;
        isAutoScoreToggledOn = currentState == State.NET && autoScore;
    }

    public void toggleNetState() {
        toggleNetState(false);
    }

    public void escalateClimb() {
        currentState = (currentState == State.CLIMB || currentState == State.PREPARE_CLIMB) ? State.CLIMB : State.PREPARE_CLIMB;
    }

    public void configureToggleStateTriggers() { // TODO: OPTIMIZE STATE TRANSITIONS
        autoLevelState.whileTrue(
                Commands.run(
                        () -> {
                            if (currentState != getAutoLevelState()) {
                                toggleAutoLevelCoralState();
                            }
                        }
                )
        );

        stowState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(climb::reset)
                        .andThen(intake::setIdleState)
                        .andThen(wrist::setStowState)
                        .andThen(new WaitUntilCommand(wrist::nearTarget))
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

        intakeOutState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setIntakeOutState)
                        .andThen(new WaitUntilCommand(() -> !intake.hasAlgae() && !intake.hasCoral()))
                        .andThen(this::setStowState)
        );

        coralStationState.onTrue(
                new InstantCommand(swerve::setCoralStationHeadingMode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(transition(Pivot.State.CORAL_STATION))
                        .andThen(intake::setIntakeState)
                        .andThen(pivot::setCoralStationState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setCoralStationState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setCoralStationState)
                        .andThen(new WaitUntilCommand(intake::atIdleState))
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
                        .andThen(new WaitUntilCommand(PhotonUtil.Color::hasCoralTargets))
                        .andThen(swerve.directMoveToObject(
                                () -> intake.hasAlgae() || intake.beamBreakBroken(),
                                PhotonUtil.Color.TargetClass.CORAL
                        ).asProxy())
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
                        .andThen(new WaitUntilCommand(PhotonUtil.Color::hasAlgaeTargets))
                        .andThen(swerve.directMoveToObject(
                                () -> intake.hasAlgae() || intake.beamBreakBroken(),
                                PhotonUtil.Color.TargetClass.ALGAE
                        ).asProxy())
                        .andThen(this::setStowState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !groundAlgaeState.getAsBoolean())
        );

        l1CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingL1Mode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(intake::setIdleState)
                        .andThen(transition(Pivot.State.L1_CORAL))
                        .andThen(pivot::setL1CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL1CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL1CoralState)
                        .until(() -> !l1CoralState.getAsBoolean())
        );

        l2CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingMode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(intake::setIdleState)
                        .andThen(transition(Pivot.State.L2_CORAL))
                        .andThen(pivot::setL2CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL2CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL2CoralState)
                        .andThen(
                                new WaitUntilCommand(() -> swerve.localizer.atScoringLocation(currentState) && atL2CoralState.getAsBoolean()) // TODO SHOP: TEST AT SCORING LOCATION STATE IN AUTO BEFORE TESTING IT HERE
                                        .andThen(this::toggleL2CoralState)
                                        .onlyIf(() -> isAutoScoreToggledOn)
                        )
                        .until(() -> !l2CoralState.getAsBoolean())
        );

        l3CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingMode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(intake::setIdleState)
                        .andThen(transition(Pivot.State.L3_CORAL))
                        .andThen(pivot::setL3CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL3CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL3CoralState)
                        .andThen(
                                new WaitUntilCommand(() -> swerve.localizer.atScoringLocation(currentState) && atL3CoralState.getAsBoolean())
                                        .andThen(this::toggleL3CoralState)
                                        .onlyIf(() -> isAutoScoreToggledOn)
                        )
                        .until(() -> !l3CoralState.getAsBoolean())
        );

        l4CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingMode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(intake::setIdleState)
                        .andThen(transition(Pivot.State.L4_CORAL))
                        .andThen(pivot::setL4CoralState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setL4CoralState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setL4CoralState)
                        .andThen(
                                new WaitUntilCommand(() -> swerve.localizer.atScoringLocation(currentState) && atL4CoralState.getAsBoolean())
                                        .andThen(this::toggleL4CoralState)
                                        .onlyIf(() -> isAutoScoreToggledOn)
                        )
                        .until(() -> !l4CoralState.getAsBoolean())
        );

        lowReefAlgaeState.onTrue(
                new InstantCommand(swerve::setReefTagOppositeHeadingMode)
                        .andThen(transition(Pivot.State.LOW_REEF_ALGAE))
                        .andThen(intake::setOuttakeState)
                        .andThen(pivot::setLowReefAlgaeState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setLowReefAlgaeState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setLowReefAlgaeState)
                        .andThen(new WaitUntilCommand(intake::atIdleState))
                        .andThen(this::setStowState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !lowReefAlgaeState.getAsBoolean())
        );

        highReefAlgaeState.onTrue(
                new InstantCommand(swerve::setReefTagHeadingMode)
                        .andThen(transition(Pivot.State.HIGH_REEF_ALGAE))
                        .andThen(intake::setIntakeState)
                        .andThen(pivot::setHighReefAlgaeState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setHighReefAlgaeState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setHighReefAlgaeState)
                        .andThen(new WaitUntilCommand(intake::atIdleState))
                        .andThen(this::setStowState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !highReefAlgaeState.getAsBoolean())
        );

        processorState.onTrue(
                new InstantCommand(swerve::setProcessorHeadingMode)
                        .andThen(intake::setIdleState)
                        .andThen(wrist::setProcessorState)
                        .andThen(elevator::setProcessorState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(pivot::setProcessorState)
                        .andThen(
                                new WaitUntilCommand(() -> swerve.localizer.atScoringLocation(currentState) && atProcessorState.getAsBoolean())
                                        .andThen(this::toggleProcessorState)
                                        .onlyIf(() -> isAutoScoreToggledOn)
                        )
                        .until(() -> !processorState.getAsBoolean())
        );

        netState.onTrue(
                new InstantCommand(swerve::setNetHeadingMode)
                        .andThen(intake::setIdleState)
                        .andThen(transition(Pivot.State.NET))
                        .andThen(pivot::setNetState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setNetState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setNetState)
                        .andThen(
                                new WaitUntilCommand(() -> swerve.localizer.atScoringLocation(currentState) && atNetState.getAsBoolean())
                                        .andThen(this::toggleNetState)
                                        .onlyIf(() -> isAutoScoreToggledOn)
                        )
                        .until(() -> !netState.getAsBoolean())
        );

        prepareClimbState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(climb::escalateClimb)
                        .andThen(intake::setIdleState)
                        .andThen(transition(Pivot.State.CLIMB))
                        .andThen(pivot::setClimbState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(elevator::setClimbState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(wrist::setClimbState)
                        .until(() -> !prepareClimbState.getAsBoolean())
        );

        climbState.onTrue(
                new InstantCommand(climb::escalateClimb)
        );
    }

    /* Each subsystem will execute their corresponding command periodically */
    public void setDefaultCommands(CommandXboxController driverXbox, CommandXboxController opXbox) {
        /* Note that X is defined as forward according to WPILib convention,
        and Y is defined as to the left according to WPILib convention.
        drive forward with left joystick negative Y (forward),
        drive left with left joystick negative X (left),
        rotate counterclockwise with right joystick negative X (left) */
        swerve.setDefaultCommand(
                swerve.driveFieldCentric(
                        elevator::getPosition,
                        driverXbox::getLeftY,
                        driverXbox::getLeftX,
                        driverXbox::getRightX,
                        driverXbox::getLeftTriggerAxis,
                        driverXbox::getRightTriggerAxis,
                        DoubleTrueTrigger.doubleTrue(driverXbox.leftTrigger(), 0.5),
                        DoubleTrueTrigger.doubleTrue(driverXbox.rightTrigger(), 0.5)
                )
        );

        elevator.setDefaultCommand(new ElevatorCommand(elevator, opXbox::getLeftX, pivot::getPosition, this));

        intake.setDefaultCommand(new IntakeCommand(intake));

        pivot.setDefaultCommand(
                new PivotCommand(pivot, () -> -opXbox.getLeftY(), elevator::getPosition, wrist::getPosition, this)
        );

        wrist.setDefaultCommand(
                new WristCommand(wrist, () -> -opXbox.getRightY(), pivot::getPosition, elevator::getPosition, this)
        );
    }

    private Command transition(Pivot.State pivotState) {
        return new ConditionalCommand(
                new InstantCommand(wrist::setStowState)
                        .andThen(new WaitUntilCommand(wrist::nearTarget)),
                new InstantCommand(wrist::setStowState)
                        .andThen(new WaitUntilCommand(wrist::nearTarget))
                        .andThen(elevator::setStowState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget)),
                () -> pivot.isAtState(pivotState)
        );
    }

    public void publishValues() {
        robotStatesPub.set(currentState.name());

        logValues();
    }

    private void logValues() {
        DogLog.log("RobotState", currentState);
    }
}
