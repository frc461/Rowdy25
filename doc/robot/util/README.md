# Utilities Documentation
The util package contains helper classes supporting subsystems and commands: vision integrations, field mathematics, control utilities, and custom triggers.
## Categories
- [Vision Utilities](VISION.md) - Limelight, PhotonVision, QuestNav wrappers and integration
- [Other Utilities](OTHER.md) - Field math, custom triggers, SysID, control helpers
## Key Utilities
**Vision**: LimelightUtil, PhotonUtil, QuestNavUtil wrap camera APIs for pose estimation and AprilTag detection.

**Field Math**: FieldUtil, RobotPoses calculate target positions (reef, algae, station, net, processor, barge); RotationUtil handles angle math.

**Control**: SysID characterizes motors; PhoenixProfiledPIDController, ProfiledExpEndController provide smooth motion profiles; GravityGainsCalculator tunes gravity compensation.

**Custom Triggers**: DoubleTrueTrigger detects simultaneous button presses; MultipleChooser extends SmartDashboard selection capabilities.
## See Also
- [Localizer](../subsystems/LOCALIZER.md) - Uses vision and FieldUtil for pose estimation
- [Pathfinder](../autos/PATHFINDER.md) - Uses FieldUtil for dynamic path generation
