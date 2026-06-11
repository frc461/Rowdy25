# Robot Documentation
The classes within the src/main/java/io/github/frc461/rowdy25/ directory are the core robot software implementation. This documentation covers the main robot lifecycle, state machine superstructure, and modular subsystems that together form Rowdy25's autonomous and teleoperated behavior.
## Core Robot Classes
- [Robot](ROBOT.md) - Main robot class and lifecycle framework
- [RobotContainer](ROBOT_CONTAINER.md) - Initialization hub for subsystems and controls
- [RobotStates](ROBOT_STATES.md) - Superstructure state machine for coordinated subsystem transitions
## Subsystems & Commands
- [Subsystems](subsystems) - Lower-level mechanical implementations (Swerve, Elevator, Pivot, Wrist, Intake, Localizer, Lights)
- [Commands](commands) - Higher-level actions and behaviors
- [Autonomous](autos) - Auto routines, pathfinding, and chooser logic
## Configuration & Support
- [Constants](constants) - Centralized configuration for motors, sensors, PID gains, and field layout
- [Utilities](util) - Vision, field math, and custom control utilities
