# Drive Commands

Swerve drivetrain commands for manual and autonomous control. All live under [`commands/drive/`](../../src/main/java/io/github/frc461/rowdy25/commands/drive/).

## Commands

- **`DriveCommand`** — Default command for the [Swerve](../subsystems/DRIVETRAIN.md) subsystem. Implements field-centric teleop drive with a configurable `DriveMode` (idle, translating, rotating / fast-rotating, branch/reef-tag/coral-station/processor/net/object auto-heading). Driver translation and rotation inputs are scaled by current elevator height for stability when extended.
- **`PathfindToPoseAvoidingReefCommand`** — Wraps PathPlanner's pathfinder so that the generated trajectory avoids the reef obstacle zones; used by every `Swerve.pathFindTo*` helper.
- **`DirectMoveToPoseCommand`** — Drives directly toward a target pose using PID without invoking the pathfinder. Used for short approaches once the robot is near its target (e.g., final vision-based alignment).

## Features

- Field-centric drive with multiple automatic heading modes
- Vision-based heading lock onto reef branches, tags, coral stations, processor, net, and detected game pieces
- Obstacle avoidance via `LocalADStar` plus the reef-avoidance wrapper
- Velocity scaling tied to elevator extension to prevent tipping

## Default Command

`DriveCommand` is installed by `RobotStates.setDefaultCommands(...)` and runs whenever no other command requires the Swerve subsystem.

## See Also

- [Swerve subsystem](../subsystems/DRIVETRAIN.md) — Owns the `DriveMode` enum and pathfinding helpers
- [Pathfinder utility](../autos/PATHFINDER.md) — Autonomous-side wrapper around PathPlanner pathfinding
