# Autonomous Commands

Autonomous-only commands for automated game-piece handling and conditional pathfinding during the 15-second auto period. These live under [`commands/auto/`](../../src/main/java/io/github/frc461/rowdy25/commands/auto/).

## Commands

- **`SearchForObjectCommand`** — Drives a search pattern with the swerve, using PhotonVision object detection to find a coral/algae on the floor, then transitions to a direct-move-to-object command once a target is acquired.
- **`FollowPathRequiringAlgaeCommand`** — Wraps a PathPlanner `FollowPathCommand` with a precondition that the intake reports algae. Used to skip an algae-scoring leg of the autonomous routine if the previous step failed to acquire algae.

## Integration

Both commands are intended to be composed into the larger `AutoEventLooper` sequence built by [AutoManager](../autos/AUTO_MANAGER.md). They can also be registered as `NamedCommands` so PathPlanner event markers trigger them at specific waypoints.

## See Also

- [AutoManager](../autos/AUTO_MANAGER.md) — Composes these commands into the final routine
- [Drive Commands](DRIVE.md) — `PathfindToPoseAvoidingReefCommand` is the planner the auto routine actually uses for every drive segment.
- [Routines](../autos/ROUTINES.md) — `AutoEventLooper` and `AutoTrigger` polling model.
