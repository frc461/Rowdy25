# RobotStates Class
The [RobotStates](../../src/main/java/io/github/frc461/rowdy25/RobotStates.java) class is a superstructure that integrates all non-drivetrain subsystems (Elevator, Pivot, Wrist, Intake, Lights) into a coordinated state machine. It manages discrete robot states and ensuring safe, ordered transitions.
## State Overview
RobotStates.State is an enum representing 25+ robot states, including:
- **Stow States**: STOW, L2_L3_L4_STOW - Safe resting positions
- **Coral Scoring**: GROUND_CORAL, L1_CORAL, L2_CORAL, L3_CORAL, L4_CORAL - Scoring on reef branches
- **Algae Handling**: GROUND_ALGAE, LOW_REEF_ALGAE, HIGH_REEF_ALGAE, PROCESSOR, NET - Algae removal and scoring
- **Intake Positions**: CORAL_STATION, CORAL_STATION_OBSTRUCTED - Pickup from stations
- **Climb**: PREPARE_CLIMB, CLIMB - Barge engagement and climbing
- **Manual**: MANUAL - Direct joystick control of subsystems
- **Output**: OUTTAKE, OUTTAKE_ALGAE, OUTTAKE_L1, INTAKE_OUT - Ejection modes
## State Transitions
The [orderedTransition()](../../src/main/java/io/github/frc461/rowdy25/RobotStates.java) method ensures safe transitions by:
- Moving subsystems in a defined sequence (e.g., stow wrist before rotating pivot through danger zones)
- Tracking elevator direction to apply correct order
- Validating state compatibility before executing transitions
## Triggers & Commands
State transitions are wired to controller triggers via configureToggleStateTriggers(), automatically scheduling coordinated commands when a state is toggled.
## See Also
- [Subsystems](subsystems) - Individual subsystem state enums that mirror RobotStates.State
- [RobotContainer](ROBOT_CONTAINER.md) - Where state triggers are configured
