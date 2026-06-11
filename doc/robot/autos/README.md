# Autonomous Documentation

This subdirectory covers autonomous routines, path management, and dynamic auto generation during the 15-second autonomous period.

## Core Components

- [AutoManager](AUTO_MANAGER.md) — Chooser-based auto builder with dynamic command generation
- [Pathfinder](PATHFINDER.md) — Utility for PathPlanner pathfinding to field elements
- [Routines](ROUTINES.md) — `AutoEventLooper` and `AutoTrigger` building blocks

## Workflow

1. **Start position selection** — Choose the starting location via SmartDashboard
2. **Scoring selection** — Select which branches / levels / algae locations to score
3. **Dynamic generation** — `AutoManager` builds a command sequence from those selections
4. **Path execution** — PathPlanner paths run with automatic heading control
5. **Event markers** — Waypoint events trigger state changes (e.g., intake enable at specific points)

## PathPlanner Integration

Paths are pre-generated in the PathPlanner UI and stored under `src/main/deploy/pathplanner/paths/`. The `LocalADStar` pathfinder can dynamically compute paths to field elements while avoiding obstacles (reef zones, wall boundaries) and is warmed up at startup from `RobotContainer`.

## See Also

- [RobotContainer](../ROBOT_CONTAINER.md) — Where `AutoManager` is instantiated and named commands are registered
- [Commands](../commands) — Auto-specific commands registered as named markers
