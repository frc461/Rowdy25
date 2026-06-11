# Vision Utilities

The classes in [`util/vision/`](../../src/main/java/io/github/frc461/rowdy25/util/vision/) wrap each camera/SLAM API used by Rowdy25 in a uniform interface that the [Localizer](../subsystems/LOCALIZER.md) can consume.

## LimelightUtil

Wrapper around the Limelight NetworkTables API. Provides:

- AprilTag pose estimation (MegaTag2 and single-tag)
- Target detection and offset vectors
- Pipeline switching and LED control
- Timestamp / latency compensation for the pose estimator

## PhotonUtil

Integration with the PhotonVision Java library:

- Multi-camera AprilTag pose estimation
- Color / ML-based coral and algae detection (used by `SearchForObjectCommand` and the object-heading drive mode)
- NetworkTables publishing for debugging

## QuestNavUtil

Backup SLAM-based localization via QuestNav. Provides an absolute pose source that is independent of AprilTags — useful when AprilTag detections become unreliable (occlusion, lighting, etc.).

## Integration

All three utilities feed into [Localizer](../subsystems/LOCALIZER.md). A SmartDashboard chooser selects the active localization strategy at runtime, and `Localizer.toggleTrustCameras()` (bound to the driver controller) provides an emergency fallback to pure odometry.

## See Also

- [Localizer](../subsystems/LOCALIZER.md) — Consumer of all vision utilities
- [`VisionConstants`](../constants/CONSTANTS.md) — Camera mount offsets and per-camera tuning
