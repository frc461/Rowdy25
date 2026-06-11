# Vision Utilities
Vision utilities wrap camera APIs for pose estimation and game piece detection.
## LimelightUtil
Wrapper for Limelight camera NetworkTable queries. Provides methods for:
- AprilTag pose estimation (MegaTag2, single-tag)
- Target detection and offset vectors
- Pipeline switching and LED control
- Latency compensation
## PhotonUtil
Integration with PhotonVision library:
- Multi-camera pose estimation
- Color-based coral/algae target detection
- Fiducial-based tracking
- Network table publishing
## QuestNav Util
Backup SLAM-based localization via QuestNav. Provides absolute positioning when AprilTag-based approaches fail.
## Integration
All three utilities are integrated into [Localizer](../subsystems/LOCALIZER.md) via configurable strategies. SmartDashboard chooser allows runtime selection of active vision system.
## See Also
- [Localizer](../subsystems/LOCALIZER.md) - Uses vision utilities for pose fusion
- [VisionConstants](../constants/CONSTANTS.md) - Camera offsets and tuning
