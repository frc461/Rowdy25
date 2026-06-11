# AutoManager Class

[AutoManager](../../src/main/java/io/github/frc461/rowdy25/autos/AutoManager.java) dynamically generates the autonomous command from a set of SmartDashboard choosers, eliminating the need for hand-written routine classes per match.

## Choosers

- **Starting Position** — `StartPosition` enum: `CUSTOM`, `DRIVER_FAR_RIGHT`, `DRIVER_CENTER_RIGHT`, `DRIVER_CENTER`, `DRIVER_CENTER_LEFT`, `DRIVER_FAR_LEFT` (each maps to a blue-alliance `Pose2d`)
- **Scoring Sequence** — Selectable per-cycle reef branches with target levels
- **Algae Selection** — Optional algae removal/scoring locations to interleave with coral cycles
- **Coral Station Override** — Forces use of a specific coral station instead of the nearest
- **General Preferences** — Initial push, ground intake fallback, etc.

The `AutoManager` constructor wires every chooser back to a regeneration callback so the autonomous command rebuilds whenever the operator changes a selection.

## Generation

Internally `generateAutoEventLooper(...)` builds a single command by:

1. Driving to the starting pose
2. Optionally executing the push routine
3. Cycling through the chosen scoring sequence, picking the most efficient coral station between cycles (`getMostEfficientCoralStation(...)`) and inserting algae actions where requested
4. Honoring final preferences (e.g., ground-intake fallback, end pose)

`getStartingPose(...)` is overloaded to derive a pose from either a `StartPosition` or a `PathPlannerPath` start.

## Public API

- `AutoManager(RobotStates robotStates)` — Constructor; registers choosers and triggers the first command build
- `Command getFinalAutoCommand()` — Returns the most recently generated autonomous command (called by `RobotContainer.getAutonomousCommand()`)

## SmartDashboard Integration

All choosers are exposed during disabled mode for operator selection. The most recently generated command is returned at `autonomousInit()`.

## See Also

- [Pathfinder](PATHFINDER.md) — Path generation utility used internally
- [Routines](ROUTINES.md) — `AutoEventLooper` and `AutoTrigger` building blocks
- [RobotContainer](../ROBOT_CONTAINER.md) — Where `AutoManager` is instantiated
