# Subsystem Commands
Subsystem-specific commands provide higher-level control for individual subsystems via position setters and state management.
## Types
- **ElevatorCommand** - Moves elevator to target position from joystick or preset state
- **PivotCommand** - Rotates pivot via joystick or state-based positioning
- **WristCommand** - Fine-tunes wrist angle via joystick or state positioning
- **IntakeCommand** - Manages intake motor speed and game piece detection
## Implementation
Each command typically:
1. Declares the target subsystem as a requirement
2. Implements initialize(), execute(), isFinished(), end()
3. Reads joystick input (for manual control) or applies preset state (for automated control)
4. Updates motor outputs via subsystem setters
5. Ends when target position reached or interrupted
## See Also
- [Subsystems](../subsystems) - Lower-level position setters these commands invoke
- [RobotStates](../ROBOT_STATES.md) - State-based command sequences
