# Autonomous Documentation

This subdirectory covers autonomous routines, path management, and dynamic auto generation during the 15-second autonomous period.

## Core Components

- [AutoManager](AUTO_MANAGER.md) — Chooser-based auto builder; dynamically recompiles the autonomous routine whenever SmartDashboard selections change.
- [Routines](ROUTINES.md) — `AutoEventLooper` and `AutoTrigger` building blocks; the polling model that lets the routine react to runtime sensor data.
- [Pathfinder](PATHFINDER.md) — A standalone PathPlanner-wrapping utility. **Currently unreferenced by production code** — kept as a reference for the geometric "close pose" math. Both `AutoManager` and the Swerve helpers route through [`PathfindToPoseAvoidingReefCommand`](../commands/DRIVE.md) instead.

## Workflow

1. **Start position selection** — Operator picks one of 6 discrete starting poses via SmartDashboard.
2. **Scoring selection** — Operator chooses an ordered list of `ScoringLocation × Level` and / or `Side` keys.
3. **Dynamic generation** — `AutoManager.generateAutoEventLooper(...)` builds a fresh `AutoEventLooper` from those selections every time anything changes.
4. **Path execution** — Each segment is a `PathfindToPoseAvoidingReefCommand` produced by a `Swerve.pathFindTo*(...)` helper; reef avoidance and elevator-height-scaled velocity are handled in-process.
5. **Polling-driven transitions** — `AutoTrigger.done()` `Trigger`s fire on the looper's `EventLoop` to schedule the next segment, optionally gated on sensor conditions like `intake.coralEntered()`.

## PathPlanner Integration

PathPlanner is used for two things in this project:

- The pose-space pathfinder (`AutoBuilder.pathfindToPose`, backed by `LocalADStar`) is warmed up at startup in `RobotContainer` so its first invocation isn't slow. `Pathfinder` is the wrapper around this; it is **not** currently invoked by autonomous routines.
- Hand-authored `.path` / `.auto` files in `src/main/deploy/pathplanner/paths/` can still be loaded and replayed; named-command markers (`OUTTAKE_MARKER`, `INTAKE_MARKER`) registered in `RobotContainer` will fire at the configured waypoints. The dynamic routine built by `AutoManager` does not depend on these markers.

## See Also

- [RobotContainer](../ROBOT_CONTAINER.md) — Where `AutoManager` is instantiated and named commands are registered.
- [Commands](../commands/README.md) — Drive and superstructure commands invoked by the auto routine segments.
