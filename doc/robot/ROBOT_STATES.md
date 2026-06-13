# RobotStates Class

The [RobotStates](../../src/main/java/io/github/frc461/rowdy25/RobotStates.java) class is a superstructure that integrates all non-drivetrain subsystems (Elevator, Pivot, Wrist, Intake, Lights) — plus the Swerve drivetrain — into a coordinated state machine. It manages discrete robot states and ensures safe, ordered transitions between them.

## State Overview

`RobotStates.State` is an enum of 22 robot states, grouped here by intent:

- **Stow** — `STOW`, `L2_L3_L4_STOW` (safe resting positions; the latter is used when transitioning between L2/L3/L4 coral scoring poses)
- **Coral scoring** — `GROUND_CORAL`, `L1_CORAL`, `L2_CORAL`, `L3_CORAL`, `L4_CORAL`
- **Algae handling** — `GROUND_ALGAE`, `LOW_REEF_ALGAE`, `HIGH_REEF_ALGAE`, `PROCESSOR`, `NET`
- **Coral station intake** — `CORAL_STATION`, `CORAL_STATION_OBSTRUCTED`
- **Climb** — `PREPARE_CLIMB`, `CLIMB`
- **Manual** — `MANUAL` (direct joystick control of superstructure)
- **Outtake / output** — `OUTTAKE`, `OUTTAKE_ALGAE`, `OUTTAKE_L1`, `INTAKE_OUT`

A `SendableChooser` (`stateChooser`) is populated with every state so operators can force a state from SmartDashboard for testing purposes.

## State Transitions

The `orderedTransition()` method ensures safe transitions by:

- Sequencing subsystem motion so mechanisms do not collide (e.g., stowing the wrist before rotating the pivot through danger zones)
- Branching on `elevator.goingDown()` / `goingThroughStow()` to apply the correct ordering depending on direction
- Validating that the target state is compatible with the current superstructure pose before scheduling commands

State setters (`setStowState()`, `setL2L3L4StowState()`, `setClimbState()`, etc.) and toggles (`toggleNetState()`, `toggleProcessorState()`, `toggleAutoLevelCoralState()`, `toggleCoralStationState()`, `toggleGroundAlgaeState()`, `toggleHighReefAlgaeState()`, `toggleLowReefAlgaeState()`, `toggleAutoHeading()`, etc.) are the primary way other classes request transitions.

## Triggers & Commands

`configureToggleStateTriggers()` wires each state to a WPILib `Trigger` that fires when the state becomes active, automatically scheduling the coordinated command chain for that state. `setDefaultCommands(driverXbox, opXbox)` installs the per-subsystem default commands used during teleop.

## Auto-Level Tracking

The class also tracks the operator-selected reef level (`FieldUtil.Reef.Level`) via `getCurrentAutoLevel()` / `setCurrentAutoLevel(...)`, which is used by pathfinding helpers and the driver's automatic scoring bindings.

## See Also

- [Subsystems](subsystems) — Individual subsystem state enums that mirror `RobotStates.State`
- [RobotContainer](ROBOT_CONTAINER.md) — Where state triggers and toggle bindings are configured
