# Pathfinder Utility

[Pathfinder](../../src/main/java/io/github/frc461/rowdy25/autos/Pathfinder.java) is a final utility class wrapping PathPlanner's `AutoBuilder.pathfindToPose` with Rowdy25-specific approach offsets and field landmark lookups.

## Responsibilities

- Computing a safe target `Pose2d` relative to a chosen field landmark (e.g., the nearest reef branch with the right approach offset)
- Returning a `Command` that pathfinds to that pose using `LocalADStar` and respects path constraints from `Constants.AutoConstants`
- Providing landmark-specific helpers (nearest algae scoring location, nearest reef branch, coral station, processor, net, barge) so the rest of the codebase doesn't need to know the underlying offsets

## Typical Helpers

Method names follow the pattern `pathFindToNearest<Landmark>()` or `pathFindTo<Landmark>()` and return PathPlanner `Command`s, e.g.:

- Nearest algae scoring location
- Nearest reef branch
- Left / right coral station
- Processor
- Net
- Barge / climb approach

The Swerve subsystem exposes the public-facing pathfinding methods (`Swerve.pathFindToNearestLeftBranch(...)`, etc.); `Pathfinder` is the underlying computation layer those helpers and `AutoManager` use.

## Integration

Used by [AutoManager](AUTO_MANAGER.md) to assemble cycles in autonomous, and by [Swerve](../subsystems/DRIVETRAIN.md) for the teleop "smart" bumper bindings.

## See Also

- [FieldUtil](../util/OTHER.md) — Source of landmark math and nearest-target queries
- [RobotPoses](../constants/ROBOT_POSES.md) — Per-landmark target poses
- [Localizer](../subsystems/LOCALIZER.md) — Provides the current robot pose used as the pathfinder's start
