package io.github.frc461.rowdy25;

/*
 * Copyright (C) 2025-present 461 Boosters FIRST, Inc. dba Westside Robotics - The Rowdy 25.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.frc461.rowdy25.commands.ElevatorCommand;
import io.github.frc461.rowdy25.commands.IntakeCommand;
import io.github.frc461.rowdy25.commands.PivotCommand;
import io.github.frc461.rowdy25.commands.WristCommand;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.subsystems.Lights;
import io.github.frc461.rowdy25.subsystems.drivetrain.Swerve;
import io.github.frc461.rowdy25.subsystems.elevator.Elevator;
import io.github.frc461.rowdy25.subsystems.intake.Intake;
import io.github.frc461.rowdy25.subsystems.pivot.Pivot;
import io.github.frc461.rowdy25.subsystems.wrist.Wrist;
import io.github.frc461.rowdy25.util.DoubleTrueTrigger;
import io.github.frc461.rowdy25.util.FieldUtil;
import io.github.frc461.rowdy25.util.vision.PhotonUtil;

import java.util.Arrays;

import dev.doglog.DogLog;

/**
 * RobotStates is an integrating component of the functional framework initialized by {@link RobotContainer}. The instance would initialize and integrate all subsystems into a superstructure of the whole robot (except the drivetrain, which has an independent state system), a software-defined construct to streamline states and action/routines for example, transitioning to a coral-scoring state.
 *
 * <p>The RobotStates class is a robot characterization class, that is, the subsystems are integrated into one defined system to streamline robot-wide actions for efficiency and organization. Many actions relevant to the robot's objectives require the coordination of the entire robotic system, hence a robot-wide state machine and robot-wide defined actions to satisfy the state machine.</p>
 *
 * @author Eugene Zhang, <a href="https://github.com/e500">GitHub</a>
 * @author Aneesh Terani, <a href="https://github.com/aterani">GitHub</a>
 *
 */
public class RobotStates {
    /**
     * An enum representing the possible various states describing the robot. Based on the robot's state (with this enum type), each subsystem integrated into the superstructure (that is, every subsystem except for the drivetrain) would update its respective state, then triggering a delegated action on that subsystem.
     */
    public enum State {
        /** The superstructure is idle. The physical position of the superstructure includes a slightly turned-down pivot with the wrist and elevator down leaning towards the center of the robot. */
        STOW,
        /** The superstructure detects possession of a coral game-piece and is prepared to score L2, L3, L4. This specialized stowed position is distinct from a regular stowed position, as the pivot and wrist are physically rotated further away from the center of the robot. This orientation improves efficiency when transitioning to a scoring state on the higher levels of the reef. */
        L2_L3_L4_STOW,
        /** The superstructure has been manipulated by joystick input. The physical positions of the robot are variable, as controlled by joystick input. */
        MANUAL,
        /** The superstructure detects possession of a coral game-piece and is actively ejecting it from possession out the front of the manipulator. The physical position of the superstructure is identical to the physical position of the robot in its previous state. */
        OUTTAKE,
        /** The superstructure detects possession of an algae game-piece and is actively ejecting it from possession out the front of the manipulator. The physical position of the superstructure is identical to the physical position of the robot in its previous state (either the {@link State#NET} or {@link State#PROCESSOR} states). */
        OUTTAKE_ALGAE,
        /** The superstructure detects possession of a coral game-piece and is actively ejecting it from possession out the front of the manipulator, but slower to adapt to scoring on the reef on L1. The physical position of the superstructure is identical to the physical position of the robot in the {@link State#L1_CORAL} state.*/
        OUTTAKE_L1,
        /** The superstructure detects possession of a coral game-piece and is actively ejecting it from possession out the back of the manipulator, to adapt to scoring on L2, L3, or L4 in the case that {@link io.github.frc461.rowdy25.subsystems.localizer.Localizer#trustCameras} is toggled false (cameras are not trustworthy). The physical position of the superstructure is identical to the physical position of the robot in its previous state (any of the {@link State#L2_CORAL}, {@link State#L3_CORAL}, or {@link State#L4_CORAL} states). */
        INTAKE_OUT,
        /** The superstructure is oriented to obtain a coral game-piece while up against the coral station. The physical position of the superstructure includes a slightly turned-down pivot with the elevator down and wrist lined up with the slope of the coral station slot to receive a coral game-piece. */
        CORAL_STATION,
        /** The superstructure is oriented to obtain a coral game-piece while one coral width's away from the coral station. The physical position of the superstructure is similar to the {@link State#CORAL_STATION} state but with a slightly extended elevator. */
        CORAL_STATION_OBSTRUCTED,
        /** The superstructure is oriented to obtain a coral game-piece on the ground. The physical position of the superstructure includes an almost-horizontal pivot with the wrist parallel to the ground ready to obtain any coral game-piece on the ground. */
        GROUND_CORAL,
        /** The superstructure is oriented to obtain an algae game-piece on the ground. The physical position of the superstructure is similar to the {@link State#GROUND_CORAL} state but with the elevator tilted upward to account for the size of the algae game-piece. */
        GROUND_ALGAE,
        /** The superstructure is oriented to line up with and be prepared to score on the reef on L1. The physical position of the superstructure is similar to the {@link State#GROUND_ALGAE} state but with the wrist more consistently lined up with the L1 scoring trough on the reef. */
        L1_CORAL,
        /** The superstructure is oriented to line up with and be prepared to score on the reef on L2. The physical position of the superstructure includes an upright pivot and unextended elevator with a wrist turned over toward the branch if {@link io.github.frc461.rowdy25.subsystems.localizer.Localizer#trustCameras} is toggled true, or down between the elevator frame for passthrough outtake (intake-out) if toggled false. */
        L2_CORAL,
        /** The superstructure is oriented to line up with and be prepared to score on the reef on L3. */ // TODO FINISH PHYSICAL DESCRIPTIONS
        L3_CORAL,
        /** The superstructure is oriented to line up with and be prepared to score on the reef on L4. */
        L4_CORAL,
        /** The superstructure is oriented to line up with and be prepared to obtain an algae game-piece on the lower side of the reef. */
        LOW_REEF_ALGAE,
        /** The superstructure is oriented to line up with and be prepared to obtain an algae game-piece on the upper side of the reef. */
        HIGH_REEF_ALGAE,
        /** The superstructure is oriented to line up with and be prepared to score in the processor. */
        PROCESSOR,
        /** The superstructure is oriented to line up with and be prepared to score in the net. */
        NET,
        /** The superstructure is oriented to line up with the barge prior to entering the climb state. */
        PREPARE_CLIMB,
        /** The superstructure is oriented in an optimized position while engaged with the barge. */
        CLIMB
    }

    /**
     * {@link Swerve} is a Phoenix-based omnidirectional drivetrain subsystem.
     */
    public final Swerve swerve = new Swerve();
    /**
     * {@link Elevator} is a TalonFX Kraken motor-based extension subsystem attached onto the pivot subsystem that extends away for rotational reach.
     */
    public final Elevator elevator = new Elevator();
    /**
     * {@link Intake} is a TalonFX Kraken motor-based subsystem attached to the wrist that grabs and collects coral or algae from the field.
     */
    public final Intake intake = new Intake();
    /**
     * {@link Pivot} is a TalonFX Kraken motor-based subsystem located near the base of the robot that rotates pitch-wise with respect to the front-to-back perspective of the robot base (plane).
     */
    public final Pivot pivot = new Pivot();
    /**
     * {@link Wrist} is a TalonFX Kraken motor-based subsystem attached to the upper end of the elevator that rotates pitch-wise with respect to the front-to-back perspective of the robot base (plane). The pivot and wrist create an extendable, doubly-jointed system.
     */
    public final Wrist wrist = new Wrist();

    /**
     * The field storing the current state of the whole robot.
     */
    private State currentState = State.STOW;
    /**
     * The field storing the current branch level that the robot will target during coral scoring.
     */
    private FieldUtil.Reef.Level currentAutoLevel = FieldUtil.Reef.Level.L4;
    /**
     * A chooser that allows selection of any desired robot {@link State} on the {@link SmartDashboard} to override the current state of the whole robot. Transition actions/routines will occur as usual for each subsystem.
     */
    private final SendableChooser<State> stateChooser = new SendableChooser<>();

    /**
     * A {@link Trigger} of the {@link State#STOW} robot state.
     *
     * <p>This field represents the condition of whether the current state of the robot {@link #currentState} matches {@link State#STOW}. As an application, the routine to transition the robot and its subsystems into the physical state corresponding to the {@link State#STOW} state is executed when {@link #currentState} becomes {@link State#STOW}, and this trigger becomes representative of true.</p>
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger stowState = new Trigger(() -> currentState == State.STOW);
    /**
     * A {@link Trigger} of the {@link State#L2_L3_L4_STOW} robot state.
     *
     * <p>This field represents the condition of whether the current state of the robot {@link #currentState} matches {@link State#L2_L3_L4_STOW}. As an application, after collecting a coral game-piece from a coral station, the {@link #currentState} is set to {@link State#L2_L3_L4_STOW} if {@link #currentAutoLevel} is {@link FieldUtil.Reef.Level#L2} or above, triggering a routine to transition the robot into physical state corresponding to the {@link State#L2_L3_L4_STOW} state.</p>
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger l2L3L4StowState = new Trigger(() -> currentState == State.L2_L3_L4_STOW);
    /**
     * A {@link Trigger} of the {@link State#OUTTAKE} robot state. // TODO FINISH FIELD APPLICATIONS
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger outtakeState = new Trigger(() -> currentState == State.OUTTAKE);
    /**
     * A {@link Trigger} of the {@link State#OUTTAKE_ALGAE} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger outtakeAlgaeState = new Trigger(() -> currentState == State.OUTTAKE_ALGAE);
    /**
     * A {@link Trigger} of the {@link State#OUTTAKE_L1} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger outtakeL1State = new Trigger(() -> currentState == State.OUTTAKE_L1);
    /**
     * A {@link Trigger} of the {@link State#INTAKE_OUT} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger intakeOutState = new Trigger(() -> currentState == State.INTAKE_OUT);
    /**
     * A {@link Trigger} of the {@link State#CORAL_STATION} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger coralStationState = new Trigger(() -> currentState == State.CORAL_STATION);
    /**
     * A {@link Trigger} of the {@link State#CORAL_STATION_OBSTRUCTED} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger coralStationObstructedState = new Trigger(() -> currentState == State.CORAL_STATION_OBSTRUCTED);
    /**
     * A {@link Trigger} of the {@link State#GROUND_CORAL} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger groundCoralState = new Trigger(() -> currentState == State.GROUND_CORAL);
    /**
     * A {@link Trigger} of the {@link State#GROUND_ALGAE} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger groundAlgaeState = new Trigger(() -> currentState == State.GROUND_ALGAE);
    /**
     * A {@link Trigger} of the {@link State#L1_CORAL} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger l1CoralState = new Trigger(() -> currentState == State.L1_CORAL);
    /**
     * A {@link Trigger} of the {@link State#L2_CORAL} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger l2CoralState = new Trigger(() -> currentState == State.L2_CORAL);
    /**
     * A {@link Trigger} of the {@link State#L3_CORAL} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger l3CoralState = new Trigger(() -> currentState == State.L3_CORAL);
    /**
     * A {@link Trigger} of the {@link State#L4_CORAL} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger l4CoralState = new Trigger(() -> currentState == State.L4_CORAL);
    /**
     * A {@link Trigger} of the {@link State#LOW_REEF_ALGAE} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger lowReefAlgaeState = new Trigger(() -> currentState == State.LOW_REEF_ALGAE);
    /**
     * A {@link Trigger} of the {@link State#HIGH_REEF_ALGAE} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger highReefAlgaeState = new Trigger(() -> currentState == State.HIGH_REEF_ALGAE);
    /**
     * A {@link Trigger} of the {@link State#PROCESSOR} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger processorState = new Trigger(() -> currentState == State.PROCESSOR);
    /**
     * A {@link Trigger} of the {@link State#NET} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger netState = new Trigger(() -> currentState == State.NET);
    /**
     * A {@link Trigger} of the {@link State#PREPARE_CLIMB} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger prepareClimbState = new Trigger(() -> currentState == State.PREPARE_CLIMB);
    /**
     * A {@link Trigger} of the {@link State#CLIMB} robot state.
     *
     * <p>The {@link Trigger} of a certain {@link State} represents the condition of whether the current state of the robot {@link #currentState} matches that state. Much of the program's automation relies on specific conditions of these triggers of each {@link State} (e.g., when the trigger becomes true or false, whenever the trigger is true or false).</p>
     */
    public final Trigger climbState = new Trigger(() -> currentState == State.CLIMB);

    /**
     * A {@link Trigger} that indicates whether the current robot state is any of the coral-scoring states. This trigger to conditionally update the robot state to the coral-scoring state that corresponds to {@link #currentAutoLevel}.
     */
    private final Trigger isListening = l1CoralState.or(l2CoralState).or(l3CoralState).or(l4CoralState);
    /**
     * A boolean that turns true when {@link #isListening} turns true, which triggers an update to the robot state. Subsequently, this boolean is reset to false.
     */
    private boolean needsUpdate = false;

    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at its target position, which defines whether the superstructure is at its current state.
     */
    public final Trigger atState = new Trigger(() -> elevator.isAtTarget() && pivot.isAtTarget() && wrist.isAtTarget());

    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#STOW} robot state.
     */
    public final Trigger atStowState = new Trigger(() -> wrist.isAtState(Wrist.State.STOW)).and(() -> elevator.isAtState(Elevator.State.STOW)).and(() -> pivot.isAtState(Pivot.State.STOW));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#L2_L3_L4_STOW} robot state.
     */
    public final Trigger atL2L3L4StowState = new Trigger(() -> wrist.isAtState(Wrist.State.L2_L3_L4_STOW)).and(() -> elevator.isAtState(Elevator.State.L2_L3_L4_STOW)).and(() -> pivot.isAtState(Pivot.State.L2_L3_L4_STOW));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#CORAL_STATION} robot state.
     */
    public final Trigger atCoralStationState = new Trigger(() -> wrist.isAtState(Wrist.State.CORAL_STATION)).and(() -> elevator.isAtState(Elevator.State.CORAL_STATION)).and(() -> pivot.isAtState(Pivot.State.CORAL_STATION));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#CORAL_STATION_OBSTRUCTED} robot state.
     */
    public final Trigger atCoralStationObstructedState = new Trigger(() -> wrist.isAtState(Wrist.State.CORAL_STATION_OBSTRUCTED)).and(() -> elevator.isAtState(Elevator.State.CORAL_STATION_OBSTRUCTED)).and(() -> pivot.isAtState(Pivot.State.CORAL_STATION_OBSTRUCTED));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#GROUND_CORAL} robot state.
     */
    public final Trigger atGroundCoralState = new Trigger(() -> wrist.isAtState(Wrist.State.GROUND_CORAL)).and(() -> elevator.isAtState(Elevator.State.GROUND_CORAL)).and(() -> pivot.isAtState(Pivot.State.GROUND_CORAL));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#GROUND_ALGAE} robot state.
     */
    public final Trigger atGroundAlgaeState = new Trigger(() -> wrist.isAtState(Wrist.State.GROUND_ALGAE)).and(() -> elevator.isAtState(Elevator.State.GROUND_ALGAE)).and(() -> pivot.isAtState(Pivot.State.GROUND_ALGAE));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#L1_CORAL} robot state.
     */
    public final Trigger atL1CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L1_CORAL)).and(() -> elevator.isAtState(Elevator.State.L1_CORAL)).and(() -> pivot.isAtState(Pivot.State.L1_CORAL));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#L2_CORAL} robot state.
     */
    public final Trigger atL2CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L2_CORAL_AT_BRANCH)).and(() -> elevator.isAtState(Elevator.State.L2_CORAL_AT_BRANCH)).and(() -> pivot.isAtState(Pivot.State.L2_CORAL_AT_BRANCH));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#L2_CORAL} robot state while one coral width's away from the reef.
     */
    public final Trigger atL2CoralOneCoralFromBranchState = new Trigger(() -> wrist.isAtState(Wrist.State.L2_CORAL_ONE_CORAL_FROM_BRANCH)).and(() -> elevator.isAtState(Elevator.State.L2_CORAL_ONE_CORAL_FROM_BRANCH)).and(() -> pivot.isAtState(Pivot.State.L2_CORAL_ONE_CORAL_FROM_BRANCH));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#L3_CORAL} robot state.
     */
    public final Trigger atL3CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L3_CORAL_AT_BRANCH)).and(() -> elevator.isAtState(Elevator.State.L3_CORAL_AT_BRANCH)).and(() -> pivot.isAtState(Pivot.State.L3_CORAL_AT_BRANCH));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#L3_CORAL} robot state while one coral width's away from the reef.
     */
    public final Trigger atL3CoralOneCoralFromBranchState = new Trigger(() -> wrist.isAtState(Wrist.State.L3_CORAL_ONE_CORAL_FROM_BRANCH)).and(() -> elevator.isAtState(Elevator.State.L3_CORAL_ONE_CORAL_FROM_BRANCH)).and(() -> pivot.isAtState(Pivot.State.L3_CORAL_ONE_CORAL_FROM_BRANCH));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#L4_CORAL} robot state.
     */
    public final Trigger atL4CoralState = new Trigger(() -> wrist.isAtState(Wrist.State.L4_CORAL_AT_BRANCH)).and(() -> elevator.isAtState(Elevator.State.L4_CORAL_AT_BRANCH)).and(() -> pivot.isAtState(Pivot.State.L4_CORAL_AT_BRANCH));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#L4_CORAL} robot state while one coral width's away from the reef.
     */
    public final Trigger atL4CoralOneCoralFromBranchState = new Trigger(() -> wrist.isAtState(Wrist.State.L4_CORAL_ONE_CORAL_FROM_BRANCH)).and(() -> elevator.isAtState(Elevator.State.L4_CORAL_ONE_CORAL_FROM_BRANCH)).and(() -> pivot.isAtState(Pivot.State.L4_CORAL_ONE_CORAL_FROM_BRANCH));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#LOW_REEF_ALGAE} robot state.
     */
    public final Trigger atLowReefAlgaeState = new Trigger(() -> wrist.isAtState(Wrist.State.LOW_REEF_ALGAE)).and(() -> elevator.isAtState(Elevator.State.LOW_REEF_ALGAE)).and(() -> pivot.isAtState(Pivot.State.LOW_REEF_ALGAE));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#HIGH_REEF_ALGAE} robot state.
     */
    public final Trigger atHighReefAlgaeState= new Trigger(() -> wrist.isAtState(Wrist.State.HIGH_REEF_ALGAE)).and(() -> elevator.isAtState(Elevator.State.HIGH_REEF_ALGAE)).and(() -> pivot.isAtState(Pivot.State.HIGH_REEF_ALGAE));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#LOW_REEF_ALGAE} or {@link State#HIGH_REEF_ALGAE} robot states.
     */
    public final Trigger atReefAlgaeState = atLowReefAlgaeState.or(atHighReefAlgaeState);
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#PROCESSOR} robot state.
     */
    public final Trigger atProcessorState = new Trigger(() -> wrist.isAtState(Wrist.State.PROCESSOR)).and(() -> elevator.isAtState(Elevator.State.PROCESSOR)).and(() -> pivot.isAtState(Pivot.State.PROCESSOR));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#NET} robot state.
     */
    public final Trigger atNetState = new Trigger(() -> wrist.isAtState(Wrist.State.NET)).and(() -> elevator.isAtState(Elevator.State.NET)).and(() -> pivot.isAtState(Pivot.State.NET));
    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to the {@link State#CLIMB} robot state.
     */
    public final Trigger atClimbState = new Trigger(() -> wrist.isAtState(Wrist.State.CLIMB)).and(() -> elevator.isAtState(Elevator.State.CLIMB)).and(() -> pivot.isAtState(Pivot.State.CLIMB));

    /**
     * A {@link Trigger} that indicates whether the current positions of each subsystem within the superstructure are at the positions that correspond to any coral-scoring robot state.
     */
    public final Trigger atAutoScoreState = atL1CoralState.or(atL2CoralState).or(atL3CoralState).or(atL4CoralState)
            .or(atL2CoralOneCoralFromBranchState).or(atL3CoralOneCoralFromBranchState).or(atL4CoralOneCoralFromBranchState);

    /**
     * A {@link NetworkTable} entry that publishes information including robot/superstructure state telemetry.
     */
    private final NetworkTable robotStatesTelemetryTable = Constants.NT_INSTANCE.getTable("RobotStates");
    /**
     * A {@link StringPublisher} that publishes the current state of the robot in the {@link #robotStatesTelemetryTable} {@link NetworkTable}.
     */
    private final StringPublisher robotStatesPub = robotStatesTelemetryTable.getStringTopic("Current Robot State").publish();

    /**
     * Constructor for {@link RobotStates}. Configures the LED strips on the robot in the {@link Lights} class (NOTE: The physical feature on the robot does not exist). Adds all {@link State}s into {@link #stateChooser} and puts the chooser onto the {@link SmartDashboard}.
     */
    public RobotStates() {
        Lights.configureLights();

        Arrays.stream(State.values()).forEach(state -> stateChooser.addOption(state.name(), state));
        stateChooser.onChange(state -> currentState = stateChooser.getSelected());
        SmartDashboard.putData("Robot State Chooser", stateChooser);
    }

    /**
     * Getter method for {@link #currentAutoLevel}. Used as a conditional to variate various automated actions based on the reef level represented by the {@link #currentAutoLevel} field.
     *
     * @return The current branch level that the robot will target during coral scoring.
     */
    public FieldUtil.Reef.Level getCurrentAutoLevel() {
        return currentAutoLevel;
    }

    /**
     * Setter method for {@link #currentAutoLevel}. Used in manual tele-op control or autonomous mode while targeting a specified coral scoring location.
     *
     * @param level The branch level to mutate the current target level to during coral scoring.
     */
    public void setCurrentAutoLevel(FieldUtil.Reef.Level level) {
        currentAutoLevel = level;
        swerve.localizer.setL1RobotScoringSettingOverride(currentAutoLevel == FieldUtil.Reef.Level.L1);
        swerve.localizer.setL2RobotScoringSettingOverride(currentAutoLevel == FieldUtil.Reef.Level.L2);
        needsUpdate = isListening.getAsBoolean();
    }

    /**
     * Determines whether the location of the robot (using localization techniques in {@link io.github.frc461.rowdy25.subsystems.localizer.Localizer}) on the field is less than a tolerance distance threshold away from the nearest scoring location of the current robot location.
     *
     * @return True if the robot is at a scoring location, false otherwise.
     */
    public boolean atScoringLocation() {
        return swerve.localizer.atScoringLocation(currentState);
    }

    /**
     * Determines whether the location of the robot (using localization techniques in {@link io.github.frc461.rowdy25.subsystems.localizer.Localizer}) on the field is less than a tolerance distance threshold away from the corresponding location of the specified robot state.
     *
     * <p>Certain robot states correspond to specific locations on the field for example, {@link State#CORAL_STATION} corresponds to the nearest of the two preset coral station locations to the current location of the robot.</p>
     *
     * @param robotState The robot state whose nearest corresponding field location (to the current robot location) is to be compared with the current robot location.
     * @return True if the robot is near the specified robot state's corresponding field location, false otherwise.
     */
    public boolean nearStateLocation(RobotStates.State robotState) {
        return swerve.localizer.nearStateLocation(robotState);
    }

    public boolean atTransitionStateLocation(RobotStates.State robotState) {
        return swerve.localizer.atTransitionStateLocation(robotState, false);
    }

    public boolean atTransitionStateLocation(RobotStates.State robotState, boolean autoTransition) {
        return swerve.localizer.atTransitionStateLocation(robotState, autoTransition);
    }

    public void setStowState() {
        currentState = State.STOW;
    }

    public void setL2L3L4StowState() {
        currentState = State.L2_L3_L4_STOW;
    }

    public void setManualState() {
        currentState = State.MANUAL;
    }

    public void setOuttakeState() {
        currentState = State.OUTTAKE;
    }

    public void setOuttakeAlgaeState() {
        currentState = State.OUTTAKE_ALGAE;
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
        if (!intake.barelyHasCoral()) {
            currentState = (currentState == State.CORAL_STATION || currentState == State.CORAL_STATION_OBSTRUCTED) && !override ? State.STOW : State.CORAL_STATION;
        }
    }

    public void toggleCoralStationObstructedState() {
        if (!intake.barelyHasCoral()) {
            currentState = currentState == State.CORAL_STATION_OBSTRUCTED ? State.STOW : State.CORAL_STATION_OBSTRUCTED;
        }
    }

    public void toggleGroundCoralState() {
        if (!intake.barelyHasCoral()) {
            currentState = currentState == State.GROUND_CORAL ? State.STOW : State.GROUND_CORAL;
        }
    }

    public void toggleGroundAlgaeState() {
        if (!intake.barelyHasCoral()) {
            currentState = currentState == State.GROUND_ALGAE ? State.STOW : State.GROUND_ALGAE;
        }
    }

    public void toggleL1CoralState(boolean override) {
        currentState = currentState == State.L1_CORAL && !override ? State.OUTTAKE_L1 : State.L1_CORAL;
    }

    public void toggleL1CoralState() {
        toggleL1CoralState(false);
    }

    public void toggleL2CoralState(boolean override) {
        currentState = currentState == State.L2_CORAL && !override ? wrist.getState() == Wrist.State.L2_CORAL_ONE_CORAL_FROM_BRANCH ? State.OUTTAKE :
                State.INTAKE_OUT : State.L2_CORAL;
    }

    public void toggleL2CoralState() {
        toggleL2CoralState(false);
    }

    public void toggleL3CoralState(boolean override) {
        currentState = currentState == State.L3_CORAL && !override ? wrist.getState() == Wrist.State.L3_CORAL_ONE_CORAL_FROM_BRANCH ? State.OUTTAKE :
                State.INTAKE_OUT : State.L3_CORAL;
    }

    public void toggleL3CoralState() {
        toggleL3CoralState(false);
    }

    public void toggleL4CoralState(boolean override) {
        currentState = currentState == State.L4_CORAL && !override ? wrist.getState() == Wrist.State.L4_CORAL_ONE_CORAL_FROM_BRANCH ? State.OUTTAKE :
                State.INTAKE_OUT : State.L4_CORAL;
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
        if (!intake.barelyHasCoral()) {
            currentState = currentState == State.LOW_REEF_ALGAE ? State.STOW : State.LOW_REEF_ALGAE;
        }
    }

    public void toggleHighReefAlgaeState() {
        if (!intake.barelyHasCoral()) {
            currentState = currentState == State.HIGH_REEF_ALGAE ? State.STOW : State.HIGH_REEF_ALGAE;
        }
    }

    public void toggleReefAlgaeState(boolean high, boolean override) {
        currentState = high
                ? currentState == State.HIGH_REEF_ALGAE && !override ? State.STOW : State.HIGH_REEF_ALGAE
                : currentState == State.LOW_REEF_ALGAE && !override ? State.STOW : State.LOW_REEF_ALGAE;
    }

    public void toggleReefAlgaeState(boolean high) {
        toggleReefAlgaeState(high, false);
    }

    public void toggleProcessorState(boolean override) {
        currentState = currentState == State.PROCESSOR && !override ? State.OUTTAKE_ALGAE : State.PROCESSOR;
    }

    public void toggleProcessorState() {
        toggleProcessorState(false);
    }

    public void toggleNetState(boolean override) {
        currentState = currentState == State.NET && !override ? State.OUTTAKE_ALGAE : State.NET;
    }

    public void toggleNetState() {
        toggleNetState(false);
    }

    public void escalateClimb() {
        currentState = (currentState == State.CLIMB || currentState == State.PREPARE_CLIMB) ? State.CLIMB : State.PREPARE_CLIMB;
    }

    public void setClimbState() {
        currentState = State.CLIMB;
    }

    private Command movePivotToPerpendicular(boolean trustCameras) {
        return new InstantCommand(pivot::setPerpendicularState)
                .andThen(new WaitUntilCommand(pivot::isAtTarget))
                .onlyIf(() -> pivot.getPosition() > 90 && !trustCameras);
    }

    private Command orderedTransition(Runnable setPivotState, Pivot.State pivotState, Runnable setElevatorState, Elevator.State elevatorState, Runnable setWristState) {
        return orderedTransition(setPivotState, pivotState, setElevatorState, elevatorState, setWristState, false);
    }

    private Command orderedTransition(Runnable setPivotState, Pivot.State pivotState, Runnable setElevatorState, Elevator.State elevatorState, Runnable setWristState, boolean fromL2L3L4Stow) {
        return new ConditionalCommand(
                new InstantCommand(wrist::setStowState)
                        .andThen(new WaitUntilCommand(wrist::nearTarget))
                        .andThen(movePivotToPerpendicular(swerve.localizer.trustCameras))
                        .andThen(
                                new InstantCommand(pivot::setStowState)
                                        .andThen(elevator::setStowState)
                                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                                        .onlyIf(() -> pivot.goingThroughStow(pivotState))
                        )
                        .andThen(setPivotState)
                        .andThen(setElevatorState)
                        .andThen(new WaitUntilCommand(elevator::nearTarget))
                        .andThen(setWristState),
                movePivotToPerpendicular(swerve.localizer.trustCameras)
                        .andThen(new InstantCommand(wrist::setStowState).unless(() -> fromL2L3L4Stow))
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
                        .alongWith(
                                new WaitUntilCommand(() -> intake.barelyHasCoral() && currentAutoLevel != FieldUtil.Reef.Level.L1)
                                        .andThen(this::setL2L3L4StowState)
                        ).until(() -> !stowState.getAsBoolean())
        );

        l2L3L4StowState.onTrue( // TODO SHOP: TEST THIS
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setIdleState)
                        .andThen(orderedTransition(
                                pivot::setL2L3L4StowState,
                                Pivot.State.L2_L3_L4_STOW,
                                elevator::setL2L3L4StowState,
                                Elevator.State.L2_L3_L4_STOW,
                                wrist::setL2L3L4StowState
                        )).alongWith(
                                new WaitUntilCommand(() -> !intake.barelyHasCoral() || currentAutoLevel == FieldUtil.Reef.Level.L1)
                                        .andThen(this::setStowState)
                        ).until(() -> !l2L3L4StowState.getAsBoolean())
        );

        outtakeState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setOuttakeState)
                        .andThen(new WaitUntilCommand(() -> !intake.barelyHasCoral()))
                        .andThen(this::setStowState)
        );

        outtakeAlgaeState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setOuttakeState)
                        .andThen(Commands.waitSeconds(0.25))
                        .andThen(this::setStowState)
        );

        outtakeL1State.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setOuttakeL1State)
                        .andThen(new WaitUntilCommand(() -> !intake.barelyHasCoral()))
                        .andThen(new WaitCommand(0.25))
                        .andThen(this::setStowState)
        );

        intakeOutState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setIntakeOutState)
                        .andThen(new WaitUntilCommand(() -> !intake.barelyHasCoral()))
                        .andThen(this::setStowState)
        );

        coralStationState.onTrue(
                new InstantCommand(swerve::setCoralStationHeadingMode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(orderedTransition(pivot::setCoralStationState, Pivot.State.CORAL_STATION, elevator::setCoralStationState, Elevator.State.CORAL_STATION, wrist::setCoralStationState))
                        .andThen(intake::setCoralIntakeState)
                        .andThen(new WaitUntilCommand(() -> intake.barelyHasCoral() && !swerve.localizer.nearStateLocation(State.CORAL_STATION)))
                        .andThen(this::setStowState)
                        .alongWith(new WaitUntilCommand(() -> !swerve.localizer.isAgainstCoralStation() && swerve.isStuck()).andThen(this::toggleCoralStationObstructedState))
                        .until(() -> !coralStationState.getAsBoolean())
        );

        coralStationObstructedState.onTrue(
                new InstantCommand(swerve::setCoralStationHeadingMode)
                        .unless(DriverStation::isAutonomousEnabled)
                        .andThen(orderedTransition(pivot::setCoralStationObstructedState, Pivot.State.CORAL_STATION_OBSTRUCTED, elevator::setCoralStationObstructedState, Elevator.State.CORAL_STATION_OBSTRUCTED, wrist::setCoralStationObstructedState))
                        .andThen(intake::setCoralIntakeState)
                        .andThen(new WaitUntilCommand(intake::barelyHasCoral))
                        .andThen(this::toggleCoralStationState)
                        .alongWith(new WaitUntilCommand(swerve.localizer::isAgainstCoralStation).andThen(() -> toggleCoralStationState(true)))
                        .until(() -> !coralStationObstructedState.getAsBoolean())
        );

        groundCoralState.onTrue(
                new InstantCommand(swerve::setObjectHeadingMode)
                        .andThen(orderedTransition(pivot::setGroundCoralState, Pivot.State.GROUND_CORAL, elevator::setGroundCoralState, Elevator.State.GROUND_CORAL, wrist::setGroundCoralState))
                        .andThen(intake::setCoralIntakeState)
                        .andThen(
                                new WaitUntilCommand(PhotonUtil.Color::hasCoralTargets)
                                        .andThen(swerve.directMoveToObject(
                                                intake::hasCoral,
                                                PhotonUtil.Color.TargetClass.CORAL
                                        ).asProxy())
                        ).raceWith(new WaitUntilCommand(intake::hasCoral))
                        .andThen(this::setStowState)
                        .until(() -> !groundCoralState.getAsBoolean())
        );

        groundAlgaeState.onTrue(
                new InstantCommand(swerve::setObjectHeadingMode)
                        .andThen(orderedTransition(pivot::setGroundAlgaeState, Pivot.State.GROUND_ALGAE, elevator::setGroundAlgaeState, Elevator.State.GROUND_ALGAE, wrist::setGroundAlgaeState))
                        .andThen(intake::setAlgaeIntakeState)
                        .andThen(
                                new WaitUntilCommand(PhotonUtil.Color::hasAlgaeTargets)
                                        .andThen(swerve.directMoveToObject(
                                                intake::algaeStuck,
                                                PhotonUtil.Color.TargetClass.ALGAE
                                        ).asProxy())
                        ).raceWith(new WaitUntilCommand(intake::algaeStuck))
                        .andThen(this::setStowState)
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
                                () -> wrist.setL2CoralState(swerve.localizer.currentRobotScoringSetting),
                                true
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
                                () -> wrist.setL3CoralState(swerve.localizer.currentRobotScoringSetting),
                                true
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
                                () -> wrist.setL4CoralState(swerve.localizer.currentRobotScoringSetting),
                                true
                        ))
                        .until(() -> !l4CoralState.getAsBoolean())
        );

        lowReefAlgaeState.onTrue(
                new InstantCommand(swerve::setReefTagOppositeHeadingMode)
                        .andThen(intake::setAlgaeIntakeState)
                        .andThen(orderedTransition(pivot::setLowReefAlgaeState, Pivot.State.LOW_REEF_ALGAE, elevator::setLowReefAlgaeState, Elevator.State.LOW_REEF_ALGAE, wrist::setLowReefAlgaeState))
                        .andThen(new WaitUntilCommand(() -> intake.atHasAlgaeState() && !swerve.localizer.nearStateLocation(State.LOW_REEF_ALGAE)))
                        .andThen(this::setStowState)
                        .until(() -> !lowReefAlgaeState.getAsBoolean())
        );

        highReefAlgaeState.onTrue(
                new InstantCommand(swerve::setReefTagHeadingMode)
                        .andThen(intake::setAlgaeIntakeState)
                        .andThen(orderedTransition(pivot::setHighReefAlgaeState, Pivot.State.HIGH_REEF_ALGAE, elevator::setHighReefAlgaeState, Elevator.State.HIGH_REEF_ALGAE, wrist::setHighReefAlgaeState))
                        .andThen(new WaitUntilCommand(() -> intake.atHasAlgaeState() && !swerve.localizer.nearStateLocation(State.LOW_REEF_ALGAE)))
                        .andThen(this::setStowState)
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

        prepareClimbState.onTrue(
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setIdleState)
                        .andThen(orderedTransition(
                                pivot::setPrepareClimbState,
                                Pivot.State.PREPARE_CLIMB,
                                elevator::setPrepareClimbState,
                                Elevator.State.PREPARE_CLIMB,
                                wrist::setPrepareClimbState)
                        ).until(() -> !prepareClimbState.getAsBoolean())
        );

        climbState.onTrue( // TODO SHOP: TEST SLOWER PIVOT
                new InstantCommand(swerve::setIdleMode)
                        .andThen(intake::setIdleState)
                        .andThen(orderedTransition(pivot::setClimbState, Pivot.State.CLIMB, elevator::setClimbState, Elevator.State.CLIMB, wrist::setClimbState))
                        .until(() -> !climbState.getAsBoolean())
                        .andThen(pivot::setNormalMotionMagicProfile)
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
