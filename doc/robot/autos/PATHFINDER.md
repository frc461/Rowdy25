# Pathfinder Utility
[Pathfinder](../../src/main/java/io/github/frc461/rowdy25/autos/Pathfinder.java) provides utilities for dynamically generating pathfinding commands to field elements.
## Methods
- pathFindToNearestAlgaeScoringLocation() - Navigate to nearest algae target
- pathFindToNearestCoralScoringLocation() - Navigate to nearest reef branch
- pathFindToCoralStation() - Navigate to coral pickup station
- pathFindToProcessor() - Navigate to processor
- pathFindToNet() - Navigate to net
- pathFindToBarge() - Navigate to climb barge
All methods calculate offsets to approach targets safely and return PathPlanner commands.
## Integration
Used by AutoManager and manual control commands to dynamically route to targets.
## See Also
- [FieldUtil](../util/OTHER.md) - Pose calculations
- [Localizer](../subsystems/LOCALIZER.md) - Provides current robot pose
