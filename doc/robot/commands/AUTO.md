# Autonomous Commands
Autonomous-only commands for automated game piece handling and pathfinding during the 15-second auto period.
## Types
- **SearchForObjectCommand** - Searches for game pieces using vision, then moves to detected target
- **FollowPathRequiringAlgaeCommand** - Pathfinding with conditional logic (e.g., only follow if algae/coral detected)
## Integration
Commands are registered with PathPlanner's NamedCommands system, allowing path waypoint markers to trigger them dynamically.
## See Also
- [AutoManager](../autos/AUTO_MANAGER.md) - Autonomous routine builder
- [Pathfinder](../autos/PATHFINDER.md) - Dynamic pathfinding to targets
