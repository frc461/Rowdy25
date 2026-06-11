# Localizer Subsystem
The [Localizer](../../src/main/java/io/github/frc461/rowdy25/subsystems/localizer/Localizer.java) manages field-relative pose estimation using odometry fusion and vision updates.
## Pose Estimation
- **Primary**: WPILib SwerveDrivePoseEstimator fuses wheel odometry with vision measurements
- **Alternate**: QuestNav absolute SLAM-based positioning
- **Cameras**: Limelight, PhotonVision AprilTag detection; selectable via SmartDashboard
## Localization Strategy
Chooser allows switching between POSE_ESTIMATOR (odometry + vision) and QUEST_NAV (absolute SLAM).
## Field Utilities
Uses [FieldUtil](../util/OTHER.md) to compute nearest scoring targets and compute offsets for autonomous pathfinding.
## Integration
- Provides pose to [Swerve](DRIVETRAIN.md) for field-relative drive commands
- Updates [PathPlanner](../autos/PATHFINDER.md) for autonomous pathfinding
- Logs telemetry via DogLog
## Tuning
Vision offsets and measurement uncertainties in VisionConstants. Trust filter weights control fusion behavior.
