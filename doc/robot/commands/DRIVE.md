# Drive Commands
Swerve drivetrain commands for manual and autonomous control.
## Types
- **DriveCommand** - Default command for joystick-based teleop drive (field-centric, heading control)
- **PathfindToPoseAvoidingReefCommand** - Autonomous pathfinding while avoiding reef zones
- **DirectMoveToPoseCommand** - Direct movement to pose without pathfinding
## Features
- Field-centric drive with configurable heading modes
- Vision-based target alignment for scoring
- Obstacle avoidance via LocalADStar pathfinder
- Smooth motion profiles via PathPlanner constraints
## Integration
DriveCommand is the default command for Swerve; always active unless interrupted by autonomous path-following or manual heading override.
