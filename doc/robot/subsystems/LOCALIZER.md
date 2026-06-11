# Localizer Subsystem

The [Localizer](../../src/main/java/io/github/frc461/rowdy25/subsystems/localizer/Localizer.java) maintains the robot's field-relative pose by fusing wheel odometry with vision corrections.

## Pose Estimation

- **Primary** — WPILib `SwerveDrivePoseEstimator` fuses wheel odometry from `Swerve` with timestamped vision measurements
- **Alternate** — QuestNav absolute SLAM-based positioning
- **Cameras** — Limelight (AprilTag MegaTag) and PhotonVision (AprilTag + object detection); see [`util/vision/`](../util/VISION.md)

## Localization Strategy

A SmartDashboard chooser selects between the pose-estimator strategy (odometry + vision) and the QuestNav strategy. The `trustCameras` flag (toggleable from the driver controller) lets the team fall back to pure odometry + intake-based outtake when vision becomes unreliable.

## Public API

- `setPoses(...)` / `setRotations(...)` / `syncRotations()` — Manual pose injection (used by driver bindings to reset at the coral stations)
- `toggleTrustCameras()` — Flip the camera-trust flag
- `getStrategyPose()` — Currently selected pose for downstream consumers

## Field Utilities

Uses [FieldUtil](../util/OTHER.md) and [RobotPoses](../constants/ROBOT_POSES.md) to compute nearest scoring targets and offsets for pathfinding.

## Integration

- Provides the field-relative pose to [Swerve](DRIVETRAIN.md) for every field-centric drive and pathfinding command
- Feeds [PathPlanner](../autos/PATHFINDER.md) the starting pose for on-the-fly pathfinding
- Logs telemetry via DogLog (see `LocalizationTelemetry`)

## Tuning

Vision mounting offsets and measurement standard deviations live in `Constants.VisionConstants` (and its `LimelightConstants`, `PhotonConstants`, `QuestNavConstants` inner classes). Standard-deviation weighting controls how aggressively vision corrects odometry.
