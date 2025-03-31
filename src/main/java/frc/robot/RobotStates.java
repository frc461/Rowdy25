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
        OUTTAKE_L1,
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
//    public final Climb climb = new Climb();
    public final Elevator elevator = new Elevator();
    public final Intake intake = new Intake();
    public final Pivot pivot = new Pivot();
    public final Wrist wrist = new Wrist();

    private State currentState = State.STOW;
    private FieldUtil.Reef.Level currentAutoLevel = FieldUtil.Reef.Level.L4;
    private final SendableChooser<State> stateChooser = new SendableChooser<>();

    public final Trigger stowState = new Trigger(() -> currentState == State.STOW);
    public final Trigger outtakeState = new Trigger(() -> currentState == State.OUTTAKE);
    public final Trigger outtakeL1State = new Trigger(() -> currentState == State.OUTTAKE_L1);
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
    public final Trigger climbState = new Trigger(() -> currentState == State.PREPARE_CLIMB);

    private final Trigger isListening = l1CoralState.or(l2CoralState).or(l3CoralState).or(l4CoralState);
    private boolean needsUpdate = false;

    public final Trigger atState = new Trigger(() -> elevator.isAtTarget() && pivot.isAtTarget() && wrist.isAtTarget());

    public final Trigger atStowState = new Trigger(() -> wrist.isAtState(Wrist.State.STOW)).and(() -> elevator.isAtState(Elevator.State.STOW)).and(() -> pivot.isAtState(Pivot.State.STOW));
    public final Trigger atCoralStationState = new Trigger(() -> wrist.isAtState(Wrist.State.CORAL_STATION)).and(() -> elevator.isAtState(Elevator.State.CORAL_STATION)).and(() -> pivot.isAtState(Pivot.State.CORAL_STATION));
    public final Trigger atGroundCoralState = new Trigger(() -> wrist.isAtState(Wrist.State.GROUND_CORAL)).and(() -> elevator.isAtState(Elevator.State.GROUND_CORAL)).and(() -> pivot.isAtState(Pivot.State.GROUND_CORAL));
    public final Trigger atGroundAlgaeState = new Trigger(() -> wrist.isAtState(Wrist.State.GROUND_ALGAE)).and(() -> elevator.isAtState(Elevator.State.GROUND_ALGAE)).and(() -> pivot.isAtState(Pivot.State.GROUND_ALGAE));
    public final Trigger atL1CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L1_CORAL)).and(() -> elevator.isAtState(Elevator.State.L1_CORAL)).and(() -> pivot.isAtState(Pivot.State.L1_CORAL));
    public final Trigger atL2CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L2_CORAL_AT_BRANCH)).and(() -> elevator.isAtState(Elevator.State.L2_CORAL_AT_BRANCH)).and(() -> pivot.isAtState(Pivot.State.L2_CORAL_AT_BRANCH));
    public final Trigger atL3CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L3_CORAL_AT_BRANCH)).and(() -> elevator.isAtState(Elevator.State.L3_CORAL_AT_BRANCH)).and(() -> pivot.isAtState(Pivot.State.L3_CORAL_AT_BRANCH));
    public final Trigger atL4CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L4_CORAL_AT_BRANCH)).and(() -> elevator.isAtState(Elevator.State.L4_CORAL_AT_BRANCH)).and(() -> pivot.isAtState(Pivot.State.L4_CORAL_AT_BRANCH));
    public final Trigger atL4PrepareCoralState = new Trigger(() -> wrist.isAtState(Wrist.State.STOW)).and(() -> elevator.isAtState(Elevator.State.STOW)).and(() -> pivot.isAtState(Pivot.State.L4_CORAL_AT_BRANCH));
    public final Trigger atLowReefAlgaeState = new Trigger(() -> wrist.isAtState(Wrist.State.LOW_REEF_ALGAE)).and(() -> elevator.isAtState(Elevator.State.LOW_REEF_ALGAE)).and(() -> pivot.isAtState(Pivot.State.LOW_REEF_ALGAE));
    public final Trigger atHighReefAlgaeState= new Trigger(() -> wrist.isAtState(Wrist.State.HIGH_REEF_ALGAE)).and(() -> elevator.isAtState(Elevator.State.HIGH_REEF_ALGAE)).and(() -> pivot.isAtState(Pivot.State.HIGH_REEF_ALGAE));
    public final Trigger atProcessorState = new Trigger(() -> wrist.isAtState(Wrist.State.PROCESSOR)).and(() -> elevator.isAtState(Elevator.State.PROCESSOR)).and(() -> pivot.isAtState(Pivot.State.PROCESSOR));
    public final Trigger atNetState = new Trigger(() -> wrist.isAtState(Wrist.State.NET)).and(() -> elevator.isAtState(Elevator.State.NET)).and(() -> pivot.isAtState(Pivot.State.NET));
    public final Trigger atClimbState = new Trigger(() -> wrist.isAtState(Wrist.State.CLIMB)).and(() -> elevator.isAtState(Elevator.State.CLIMB)).and(() -> pivot.isAtState(Pivot.State.CLIMB));

    public final Trigger atAutoScoreState = atL1CoralState.or(atL2CoralState).or(atL3CoralState).or(atL4CoralState);

    private final NetworkTable robotStatesTelemetryTable = Constants.NT_INSTANCE.getTable("RobotStates");
    private final StringPublisher robotStatesPub = robotStatesTelemetryTable.getStringTopic("Current Robot State").publish();

    public RobotStates() {
        Lights.configureLights();

        Arrays.stream(State.values()).forEach(state -> stateChooser.addOption(state.name(), state));
        stateChooser.onChange(state -> currentState = stateChooser.getSelected());
        SmartDashboard.putData("Robot State Chooser", stateChooser);
    }

    public FieldUtil.Reef.Level getCurrentAutoLevel() {
        return currentAutoLevel;
    }

    public void setCurrentAutoLevel(FieldUtil.Reef.Level level) {
        currentAutoLevel = level;
        swerve.localizer.setL1RobotScoringSettingOverride(currentAutoLevel == FieldUtil.Reef.Level.L1);
        needsUpdate = isListening.getAsBoolean();
    }

    public boolean atScoringLocation() {
        return swerve.localizer.atScoringLocation(currentState);
    }

    public boolean atTransitionStateLocation(RobotStates.State robotState) {
        return swerve.localizer.atTransitionStateLocation(robotState);
    }

    public boolean nearStateLocation(RobotStates.State robotState) {
        return swerve.localizer.nearStateLocation(robotState);
    }

    public void setStowState() {
        currentState = State.MANUAL;
        currentState = State.STOW;
    }

    public void setManualState() {
        currentState = State.MANUAL;
    }

    public void setOuttakeState() {
        currentState = State.OUTTAKE;
    }

    public void setOuttakeL1State() {
        currentState = State.OUTTAKE_L1;
    }

    public void setIntakeOutState() {
        currentState = State.INTAKE_OUT;
    }

    public void toggleCoralStationState() {
        toggleCoralStationState(false);
    }

    public void toggleCoralStationState(boolean override) {
        currentState = currentState == State.CORAL_STATION && !override ? State.STOW : State.CORAL_STATION;
    }

    public void toggleGroundCoralState() {
        currentState = currentState == State.GROUND_CORAL ? State.STOW : State.GROUND_CORAL;
    }

    public void toggleGroundAlgaeState() {
        currentState = currentState == State.GROUND_ALGAE ? State.STOW : State.GROUND_ALGAE;
    }

    public void toggleL1CoralState(boolean override) {
        currentState = currentState == State.L1_CORAL && !override ? State.OUTTAKE_L1 : State.L1_CORAL;
    }

    public void toggleL1CoralState() {
        toggleL1CoralState(false);
    }

    public void toggleL2CoralState(boolean override) {
        currentState = currentState == State.L2_CORAL && !override ? State.INTAKE_OUT : State.L2_CORAL;
    }

    public void toggleL2CoralState() {
        toggleL2CoralState(false);
    }

    public void toggleL3CoralState(boolean override) {
        currentState = currentState == State.L3_CORAL && !override ? State.INTAKE_OUT : State.L3_CORAL;
    }

    public void toggleL3CoralState() {
        toggleL3CoralState(false);
    }

    public void toggleL4CoralState(boolean override) {
        currentState = currentState == State.L4_CORAL && !override ? State.INTAKE_OUT : State.L4_CORAL;
    }

    public void toggleL4CoralState() {
        toggleL4CoralState(false);
    }

    public void toggleAutoLevelCoralState(boolean override) {
        switch (currentAutoLevel) {
            case L1 -> toggleL1CoralState(override);
            case L2 -> toggleL2CoralState(override);
            case L3 -> toggleL3CoralState(override);
            case L4 -> toggleL4CoralState(override);
        }
    }

    public void toggleAutoLevelCoralState() {
        toggleAutoLevelCoralState(false);
    }

    public void toggleLowReefAlgaeState() {
        currentState = currentState == State.LOW_REEF_ALGAE ? State.STOW : State.LOW_REEF_ALGAE;
    }

    public void toggleHighReefAlgaeState() {
        currentState = currentState == State.HIGH_REEF_ALGAE ? State.STOW : State.HIGH_REEF_ALGAE;
    }

    public void toggleNearestReefAlgaeState(boolean high, boolean override) {
        currentState = high
                ? currentState == State.HIGH_REEF_ALGAE && !override ? State.STOW : State.HIGH_REEF_ALGAE
                : currentState == State.LOW_REEF_ALGAE && !override ? State.STOW : State.LOW_REEF_ALGAE;
    }

    public void toggleNearestReefAlgaeState(boolean high) {
        toggleNearestReefAlgaeState(high, false);
    }

    public void toggleProcessorState(boolean override) {
        currentState = currentState == State.PROCESSOR && !override ? State.OUTTAKE : State.PROCESSOR;
    }

    public void toggleProcessorState() {
        toggleProcessorState(false);
    }

    public void toggleNetState(boolean override) {
        currentState = currentState == State.NET && !override ? State.OUTTAKE : State.NET;
    }

    public void toggleNetState() {
        toggleNetState(false);
    }

    public void escalateClimb() {
        currentState = (currentState == State.CLIMB || currentState == State.PREPARE_CLIMB) ? State.CLIMB : State.PREPARE_CLIMB;
    }

    private Command movePivotToPerpendicular() {
        return new InstantCommand(pivot::setPerpendicularState)
                .andThen(new WaitUntilCommand(pivot::isAtTarget))
                .onlyIf(() -> pivot.getPosition() > 90);
    }

    private Command orderedTransition(Runnable setPivotState, Pivot.State pivotState, Runnable setElevatorState, Elevator.State elevatorState, Runnable setWristState) {
        return new ConditionalCommand(
                new InstantCommand(wrist::setStowState) // TODO SHOP: TEST CHECK IF MOVE THROUGH STOW E.G., L4 TO GROUND
                        .andThen(movePivotToPerpendicular()) // TODO: DOUBLE-SIDED REEF SCORING
                        .andThen(
                                new InstantCommand(pivot::setStowState)
                                        .andThen(elevator::setStowState)
                                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                                        .onlyIf(() -> pivot.goingThroughStow(pivotState))
                        )
                        .andThen(setElevatorState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(setPivotState)
                        .andThen(setWristState),
                movePivotToPerpendicular()
                        .andThen(wrist::setStowState)
                        .andThen(setPivotState)
                        .andThen(new WaitUntilCommand(pivot::nearTarget))
                        .andThen(setElevatorState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(setWristState),
                () -> elevator.goingDown(elevatorState)
        );
    }

    public void configureToggleStateTriggers() {
        isListening.and(() -> needsUpdate).onTrue(
                new InstantCommand(this::toggleAutoLevelCoralState)
                        .andThen(() -> needsUpdate = false)
        );

        stowState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setIdleState)
                        .andThen(orderedTransition(pivot::setStowState, Pivot.State.STOW, elevator::setStowState, Elevator.State.STOW, wrist::setStowState))
        );

        outtakeState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setOuttakeState)
                        .andThen(new WaitUntilCommand(() -> !intake.hasAlgae() && !intake.hasCoral()))
                        .andThen(this::setStowState)
        );

        outtakeL1State.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setOuttakeL1State)
                        .andThen(new WaitUntilCommand(() -> !intake.hasAlgae() && !intake.hasCoral()))
                        .andThen(this::setStowState)
        );

        intakeOutState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(() -> wrist.setL4CoralObstructedState(!swerve.localizer.isAgainstReefWall() && !swerve.localizer.trustCameras))
                        .andThen(new WaitUntilCommand(wrist::isAtTarget))
                        .andThen(intake::setIntakeOutState)
                        .andThen(new WaitUntilCommand(() -> !intake.hasAlgae() && !intake.hasCoral()))
                        .andThen(this::setStowState)
        );

        coralStationState.onTrue(
                new InstantCommand(swerve::setCoralStationHeadingMode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(orderedTransition(pivot::setCoralStationState, Pivot.State.CORAL_STATION, elevator::setCoralStationState, Elevator.State.CORAL_STATION, wrist::setCoralStationState))
                        .andThen(intake::setCoralIntakeState)
                        .andThen(new WaitUntilCommand(intake::atIdleState))
                        .andThen(this::setStowState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !coralStationState.getAsBoolean())
        );

        groundCoralState.onTrue(
                new InstantCommand(swerve::setObjectHeadingMode)
                        .andThen(orderedTransition(pivot::setGroundCoralState, Pivot.State.GROUND_CORAL, elevator::setGroundCoralState, Elevator.State.GROUND_CORAL, wrist::setGroundCoralState))
                        .andThen(intake::setCoralIntakeState)
                        .andThen(new WaitUntilCommand(PhotonUtil.Color::hasCoralTargets))
                        .andThen(swerve.directMoveToObject(
                                () -> intake.hasAlgae() || intake.hasCoral(),
                                PhotonUtil.Color.TargetClass.CORAL
                        ).asProxy())
                        .andThen(this::setStowState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !groundCoralState.getAsBoolean())
        );

        groundAlgaeState.onTrue(
                new InstantCommand(swerve::setObjectHeadingMode)
                        .andThen(orderedTransition(pivot::setGroundAlgaeState, Pivot.State.GROUND_ALGAE, elevator::setGroundAlgaeState, Elevator.State.GROUND_ALGAE, wrist::setGroundAlgaeState))
                        .andThen(intake::setAlgaeIntakeState)
                        .andThen(new WaitUntilCommand(PhotonUtil.Color::hasAlgaeTargets))
                        .andThen(swerve.directMoveToObject(
                                () -> intake.hasAlgae() || intake.hasCoral(),
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
                        .andThen(orderedTransition(pivot::setL1CoralState, Pivot.State.L1_CORAL, elevator::setL1CoralState, Elevator.State.L1_CORAL, wrist::setL1CoralState))
                        .until(() -> !l1CoralState.getAsBoolean())
        );

        l2CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingMode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(intake::setIdleState)
                        .andThen(orderedTransition(
                                () -> pivot.setL2CoralState(swerve.localizer.currentRobotScoringSetting),
                                pivot.getL2State(swerve.localizer.currentRobotScoringSetting),
                                () -> elevator.setL2CoralState(swerve.localizer.currentRobotScoringSetting),
                                elevator.getL2State(swerve.localizer.currentRobotScoringSetting),
                                () -> wrist.setL2CoralState(swerve.localizer.currentRobotScoringSetting)
                        )).until(() -> !l2CoralState.getAsBoolean())
        );

        l3CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingMode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(intake::setIdleState)
                        .andThen(orderedTransition(
                                () -> pivot.setL3CoralState(swerve.localizer.currentRobotScoringSetting),
                                pivot.getL3State(swerve.localizer.currentRobotScoringSetting),
                                () -> elevator.setL3CoralState(swerve.localizer.currentRobotScoringSetting),
                                elevator.getL3State(swerve.localizer.currentRobotScoringSetting),
                                () -> wrist.setL3CoralState(swerve.localizer.currentRobotScoringSetting)
                        )).until(() -> !l3CoralState.getAsBoolean())
        );

        l4CoralState.onTrue(
                new InstantCommand(swerve::setBranchHeadingMode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(intake::setIdleState)
                        .andThen(orderedTransition(
                                () -> pivot.setL4CoralState(swerve.localizer.currentRobotScoringSetting),
                                pivot.getL4State(swerve.localizer.currentRobotScoringSetting),
                                () -> elevator.setL4CoralState(swerve.localizer.currentRobotScoringSetting),
                                elevator.getL4State(swerve.localizer.currentRobotScoringSetting),
                                () -> wrist.setL4CoralState(swerve.localizer.currentRobotScoringSetting)
                        ))
                        .until(() -> !l4CoralState.getAsBoolean())
        );

        lowReefAlgaeState.onTrue(
                new InstantCommand(swerve::setReefTagOppositeHeadingMode)
                        .andThen(intake::setAlgaeIntakeState)
                        .andThen(orderedTransition(pivot::setLowReefAlgaeState, Pivot.State.LOW_REEF_ALGAE, elevator::setLowReefAlgaeState, Elevator.State.LOW_REEF_ALGAE, wrist::setLowReefAlgaeState))
                        .andThen(new WaitUntilCommand(intake::atIdleState))
                        .andThen(this::setStowState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !lowReefAlgaeState.getAsBoolean())
        );

        highReefAlgaeState.onTrue(
                new InstantCommand(swerve::setReefTagHeadingMode)
                        .andThen(intake::setAlgaeIntakeState)
                        .andThen(orderedTransition(pivot::setHighReefAlgaeState, Pivot.State.HIGH_REEF_ALGAE, elevator::setHighReefAlgaeState, Elevator.State.HIGH_REEF_ALGAE, wrist::setHighReefAlgaeState))
                        .andThen(new WaitUntilCommand(intake::atIdleState))
                        .andThen(this::setStowState)
                        .onlyIf(() -> !intake.hasAlgae() && !intake.hasCoral())
                        .until(() -> !highReefAlgaeState.getAsBoolean())
        );

        processorState.onTrue(
                new InstantCommand(swerve::setProcessorHeadingMode)
                        .andThen(intake::setIdleState)
                        .andThen(orderedTransition(pivot::setProcessorState, Pivot.State.PROCESSOR, elevator::setProcessorState, Elevator.State.PROCESSOR, wrist::setProcessorState))
                        .until(() -> !processorState.getAsBoolean())
        );

        netState.onTrue(
                new InstantCommand(swerve::setNetHeadingMode)
                        .andThen(intake::setIdleState)
                        .andThen(orderedTransition(pivot::setNetState, Pivot.State.NET, elevator::setNetState, Elevator.State.NET, wrist::setNetState))
                        .until(() -> !netState.getAsBoolean())
        );

        climbState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setIdleState)
                        .andThen(orderedTransition(pivot::setClimbState, Pivot.State.CLIMB, elevator::setClimbState, Elevator.State.CLIMB, wrist::setClimbState))
                        .until(() -> !climbState.getAsBoolean())
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

    public void publishValues() {
        robotStatesPub.set(currentState.name());

        logValues();
    }

    private void logValues() {
        DogLog.log("RobotState", currentState);
    }
}
