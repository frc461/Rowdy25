# Autonomous Documentation
This subdirectory covers autonomous routines, path management, and dynamic auto generation during the 15-second autonomous period.
## Core Components
- [AutoManager](AUTO_MANAGER.md) - Chooser-based auto builder with dynamic command generation
- [Pathfinder](PATHFINDER.md) - Utility for PathPlanner pathfinding to field elements
- [Routines](ROUTINES.md) - Autonomous helper logic and event handling
## Workflow
1. **Start Position Selection** - Choose starting location via SmartDashboard
2. **Scoring Selection** - Select which branches/locations to score
3. **Dynamic Generation** - AutoManager builds a command sequence based on selections
4. **Path Execution** - PathPlanner paths are followed with automatic heading control
5. **Event Markers** - Waypoint events trigger state changes (e.g., intake enable at specific points)
## PathPlanner Integration
Paths are pre-generated in the PathPlanner UI and stored in src/main/deploy/pathplanner/paths/. The LocalADStar pathfinder can dynamically compute paths to unreachable field elements while avoiding obstacles (reef zones, walls).
## See Also
- [RobotContainer](../ROBOT_CONTAINER.md) - AutoManager is instantiated here
- [Commands](../commands) - Auto commands registered as named markers
