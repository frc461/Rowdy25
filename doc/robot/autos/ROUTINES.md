# Autonomous Routines & Events
Autonomous helper classes manage path execution, event triggering, and state transitions during auto.
## Components
- **AutoEventLooper** - Monitors path events and triggers associated commands
- **AutoTrigger** - Represents scheduled actions at path waypoints
## Workflow
1. Path executes via PathPlanner/LocalADStar
2. Waypoint markers (e.g., INTAKE_MARKER) trigger event handlers
3. AutoEventLooper executes registered commands
4. RobotStates transitions update superstructure (elevator, pivot, wrist)
5. Next path segment or path end occurs
## Named Commands
Commands are registered in RobotContainer.configurePathPlannerNamedCommands() with marker names. When a path reaches a marker, the associated command is scheduled.
## See Also
- [PathPlanner Documentation](https://pathplanner.dev/)
- [RobotContainer](../ROBOT_CONTAINER.md) - Named command registration
