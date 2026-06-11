# Drivetrain (Swerve) Subsystem

The [Swerve](../../src/main/java/io/github/frc461/rowdy25/subsystems/drivetrain/Swerve.java) class extends CTRE's [SwerveDrivetrain](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/swerve/SwerveDrivetrain.html) and manages omnidirectional movement through four swerve modules.

## Hardware

- **Drive motors** — Phoenix 6 TalonFX (Kraken X60) per module
- **Steer motors** — Phoenix 6 TalonFX per module
- **Steer feedback** — CTRE CANcoder absolute encoder per module
- **Music** — CTRE `Orchestra` playing through the drive/steer motors (see [Song.java](../../src/main/java/io/github/frc461/rowdy25/subsystems/drivetrain/Song.java))

## Drive Modes

`Swerve.DriveMode` defines 12 modes that the default `DriveCommand` selects between:

`IDLE`, `ROTATING`, `FAST_ROTATING`, `TRANSLATING`, `BRANCH_HEADING`, `BRANCH_L1_HEADING`, `REEF_TAG_HEADING`, `REEF_TAG_OPPOSITE_HEADING`, `OBJECT_HEADING`, `CORAL_STATION_HEADING`, `PROCESSOR_HEADING`, `NET_HEADING`.

Auto-heading modes use PID to track a field-relative target angle (reef branch, tag, coral station, processor, net, or detected game piece).

## Control Methods

- `driveFieldCentric(...)` — Field-centric chassis-speed request used by `DriveCommand`
- `directMoveToObject(...)` — Drive directly toward a vision-detected object
- `forceStop()`, `setIdleMode()`, `setTranslatingMode()`, `setRotatingMode()`, `setFastRotatingMode()` — Mode setters used by triggers
- `setBranchHeadingMode()`, `setBranchHeadingL1Mode()`, `setReefTagHeadingMode()`, `setReefTagOppositeHeadingMode()`, `setObjectHeadingMode()`, `setCoralStationHeadingMode()`, `setProcessorHeadingMode()`, `setNetHeadingMode()` — Auto-heading mode setters
- `toggleAutoHeading()` — Toggles whether automatic heading tracking is active
- `pushAlliancePartnerOut()` — Programmed push routine for autonomous

### Pathfinding helpers

These return `Command`s built on PathPlanner's `LocalADStar` and the [PathfindToPoseAvoidingReefCommand](../commands/DRIVE.md):

- `pathFindToLeftCoralStation(...)`, `pathFindToRightCoralStation(...)`
- `pathFindToLeftCoralStationGroundIntakeCoral(...)`, `pathFindToRightCoralStationGroundIntakeCoral(...)`
- `pathFindToNearestLeftBranch(...)`, `pathFindToNearestRightBranch(...)`, `pathFindToScoringLocation(...)`
- `pathFindToNearestAlgaeOnReef(...)`, `pathFindToAlgaeOnReef(...)`
- `pathFindToNet(robotStates, randomized)`, `pathFindToProcessor(...)`

## Localization

The Swerve subsystem owns a [Localizer](LOCALIZER.md) instance that fuses wheel odometry with vision corrections and exposes the field-relative pose used by every pathfinding / heading helper.

## Default Command

[DriveCommand](../commands/DRIVE.md) is installed as the default command; it reads driver joystick input and applies the active `DriveMode`.

## Integration

- PathPlanner `AutoBuilder` + `LocalADStar` for autonomous and on-the-fly pathfinding
- [Localizer](LOCALIZER.md) for AprilTag / QuestNav-based pose updates
- [RobotStates](../ROBOT_STATES.md) drives mode toggles via controller bindings
