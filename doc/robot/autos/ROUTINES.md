# Autonomous Routines & Events

The classes in [`autos/routines/`](../../src/main/java/io/github/frc461/rowdy25/autos/routines/) implement the event-driven scaffolding that `AutoManager` uses to glue PathPlanner paths together with superstructure state changes.

## Components

- **`AutoEventLooper`** — Schedules a sequence of `AutoTrigger`s that fire as the autonomous routine progresses. Each loop iteration checks whether the next trigger's preconditions are met (path completion, sensor state, timing) and, if so, schedules the trigger's command.
- **`AutoTrigger`** — Represents a single scheduled action: a `Command` plus the precondition that gates it. Typical actions include scoring at the current branch, switching to coral-station intake, or toggling auto-heading.

## Workflow

1. A PathPlanner path executes via `LocalADStar` / `FollowPathCommand`.
2. Waypoint markers (e.g., `INTAKE_MARKER`, `OUTTAKE_MARKER`) fire and call the matching `NamedCommand` registered in `RobotContainer`.
3. `AutoEventLooper` polls registered `AutoTrigger`s and schedules their commands when conditions are met.
4. `RobotStates` transitions update the superstructure (elevator, pivot, wrist, intake) in lockstep with path progress.
5. The next path segment runs, or the routine ends.

## Named Commands

Named commands are registered in `RobotContainer.configurePathPlannerNamedCommands()` and bound to PathPlanner waypoint marker strings. Rowdy25 currently registers:

- `Constants.AutoConstants.OUTTAKE_MARKER` → `robotStates::toggleAutoLevelCoralState`
- `Constants.AutoConstants.INTAKE_MARKER` → `robotStates::toggleCoralStationState`

## See Also

- [PathPlanner Documentation](https://pathplanner.dev/)
- [AutoManager](AUTO_MANAGER.md) — Builds the looper from chooser selections
- [RobotContainer](../ROBOT_CONTAINER.md) — Where `NamedCommands` are registered
