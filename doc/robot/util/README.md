# Utilities Documentation

The `util` package contains helper classes supporting subsystems and commands: vision integrations, field mathematics, control utilities, and custom triggers.

## Categories

- [Vision Utilities](VISION.md) — Limelight, PhotonVision, and QuestNav wrappers
- [Other Utilities](OTHER.md) — Field math, custom triggers, SysID, control helpers

## At a Glance

- **Vision** — `LimelightUtil`, `PhotonUtil`, `QuestNavUtil` wrap their respective camera APIs for pose estimation and target detection
- **Field math** — `FieldUtil` computes reef branch, coral station, algae, processor, net, and barge poses (paired with [`RobotPoses`](../constants/ROBOT_POSES.md)); `RotationUtil` handles angle math; `EquationUtil` provides polynomial / interpolation helpers
- **Control & characterization** — `SysID` for motor characterization; `PhoenixProfiledPIDController` and `ProfiledExpEndController` for smooth motion profiles; `GravityGainsCalculator` for tuning gravity compensation
- **Triggers & misc** — `DoubleTrueTrigger` for simultaneous-press detection; `MultipleChooser` for multi-select SmartDashboard choosers; `EstimatedRobotPose` for timestamped vision pose results; `MacAddress` for identity lookup; `Elastic` and `DogLog` helpers for telemetry

## See Also

- [Localizer](../subsystems/LOCALIZER.md) — Consumes the vision utilities and `FieldUtil` for pose estimation
- [Drive Commands](../commands/DRIVE.md) — `PathfindToPoseAvoidingReefCommand` uses `FieldUtil`, `EquationUtil`, and `RotationUtil` for its reef-avoidance math.
