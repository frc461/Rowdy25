# AutoManager Class
[AutoManager](../../src/main/java/io/github/frc461/rowdy25/autos/AutoManager.java) dynamically generates autonomous routines based on SmartDashboard selections.
## Selections
- **Starting Position** - 5 starting zones along the field
- **Scoring Sequence** - Which branches, algae locations, and scoring targets
- **General Preferences** - Leave community, auto balance, etc.
## Generation
AutoManager builds a command sequence from selections:
1. Navigate to starting pose
2. Execute branch/algae/processor/net scoring in chosen order
3. Apply general preferences (e.g., leave community, pathfind to barge)
## SmartDashboard Integration
Displays choosers for all selections during disabled mode. Built command is sent to robot at auto start.
## See Also
- [Pathfinder](PATHFINDER.md) - Path generation utilities
- [RobotContainer](../ROBOT_CONTAINER.md) - Where AutoManager is instantiated
