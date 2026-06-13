# Constants Class

[`Constants`](../../src/main/java/io/github/frc461/rowdy25/constants/Constants.java) is the project's single configuration namespace. Every tunable value, motor ID, sensor port, mount transform, and lookup table reads from it. Its `public static` fields are *not* `final` — they are rebound at startup by [`RobotIdentity.initializeConstants()`](ROBOT_IDENTITY.md) from a variant class under `constants/variants/`. That mutability is what enables per-robot dispatch.

## Top-Level Fields

| Field | Type | Role |
| --- | --- | --- |
| `IDENTITY` | `RobotIdentity` | Set by `RobotIdentity.initializeConstants()` so downstream code can branch on which physical robot is running. |
| `CAN_BUS` | `CANBus` | The non-default CAN bus name (CANivore). |
| `NT_INSTANCE` | `NetworkTableInstance` | The NT instance used by every telemetry class and PhotonVision wrapper. |
| `ALLIANCE_SUPPLIER` | `Supplier<DriverStation.Alliance>` | A cached supplier of the current alliance; used everywhere that needs alliance flipping. The indirection through a supplier lets test/sim variants stub the alliance without driver-station emulation. |
| `BLUE_DEFAULT_ROTATION`, `RED_DEFAULT_ROTATION` | `Rotation2d` | Alliance-perspective "field forward" headings passed to `Swerve.setOperatorPerspectiveForward(...)`. |
| `ROBOT_LENGTH_WITH_BUMPERS`, `ROBOT_WIDTH_WITH_BUMPERS` | `Measure<Distance>` | Chassis dimensions used by reef-avoidance math (see [`DRIVE.md`](../commands/DRIVE.md) § 1.2). |
| `MAX_VEL`, `MAX_CONTROLLED_VEL`, `MAX_ANGULAR_VEL`, `MAX_CONTROLLED_ANGULAR_VEL`, `MAX_ACCEL`, `MAX_ANGULAR_ACCEL`, `MAX_CONTROLLED_ACCEL` | `double` or `DoubleUnaryOperator` | Top-level kinematic limits. The `CONTROLLED_*` variants are functions of elevator height — they're how the chassis decelerates automatically when the elevator extends (anti-tip). |
| `DEADBAND` | `double` | Universal joystick deadband (~5% of stick travel). |
| `CENTER_OF_LEFT/RIGHT_CORAL_STATION`, `FAR_LEFT/RIGHT_CORAL_STATION` | `Function<Supplier<Alliance>, Pose2d>` | Pose lookups for coral-station anchors; take the alliance supplier so the same call resolves to either color's station. |
| `SERVO_HUB_ID`, `SERVO_HUB` | int / `ServoHub` | REV ServoHub identification and instance for pivot ratchet servos. |
| `ONE_MILLION` | `double` | Constant used by the configurator profile for unlimited motion. |

## Nested Configuration Classes

### `AutoConstants`

Centralizes autonomous routing constants:

- `INTAKE_MARKER`, `OUTTAKE_MARKER`, `ALGAE_CHECK_MARKER` — PathPlanner waypoint event names matched by `RobotContainer.configurePathPlannerNamedCommands()` and by [`FollowPathRequiringAlgaeCommand`](../commands/AUTO.md).
- `PATH_CONSTRAINTS` — `PathConstraints(maxVel, maxAccel, maxAngVel, maxAngAccel)` passed to `Pathfinder.pathFindToPose` / `AutoBuilder.pathfindToPose`.
- `ROBOT_CONFIG` — PathPlanner's `RobotConfig` (mass, MOI, wheel COF, drivetrain config). Used by `Swerve.AutoBuilder.configure(...)` and by `FollowPathRequiringAlgaeCommand`.
- `TRANSLATION_TOLERANCE_TO_ACCEPT`, `TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE`, `TRANSLATION_TOLERANCE_TO_TRANSITION`, `TRANSLATION_TOLERANCE_TO_TRANSITION_AUTO`, `DEGREE_TOLERANCE_TO_ACCEPT` — the three nested tolerance bands documented in [`LOCALIZER.md`](../subsystems/LOCALIZER.md) and consumed by every drive command's termination check.
- `OBJECT_SEARCH_DEGREE_SLANT` — angle offset used by the legacy object-search routine.

### `VisionConstants`

The top-level fields are the σ functions used by `addVisionMeasurement`:

- `ODOM_STD_DEV` — Matrix3 for the odometry process noise.
- `VISION_STD_DEV_MULTITAG_FUNCTION` — `DoubleFunction<Matrix>` that maps the closest-tag distance to a covariance matrix; smaller σ at close range, larger σ at long range.
- `VISION_STD_DEV_FUNCTION` — Same shape but for single-tag (PnP-disambiguated) measurements. Always larger σ than the multi-tag function at the same distance.
- `PROXIMITY_SENSOR_DIO_PORT` — DIO port for the algae-side proximity sensor (currently unused in production).

#### `LimelightConstants`

`LIMELIGHT_NT_NAME` plus `LL_FORWARD/RIGHT/UP/ROLL/PITCH/YAW` — the camera's mount transform decomposed into six scalars. Pushed to the Limelight via NetworkTables in `LimelightUtil.configureRobotToCameraOffset()`. `LL_MAX_TAG_CLEAR_DIST` is the range gate used by `isTagClear()` (see [`VISION.md`](../util/VISION.md)).

#### `PhotonConstants`

Per-camera mount transforms (`BW_TOP_RIGHT_*`, `BW_TOP_LEFT_*`, `BW_BACK_*`, `COLOR_*`) using the same six-scalar decomposition. Plus:

- `BW_MAX_TAG_CLEAR_DIST` — the Photon-side range gate (analogous to the Limelight one).
- `OBJECT_TARGET_PITCH` — pixel pitch at which the color camera target is "centered enough" for the L1 trough offset to fire.

#### `QuestNavConstants`

Mount transform + `TRANSLATION_ERROR_TOLERANCE`, `ROTATION_ERROR_TOLERANCE` (for the calibration handshake), and `MIN_TAG_DIST_TO_BE_FAR` (the threshold used by `Localizer.updateQuestNavPose()` to unlatch the "calibrated near" flag).

### `ElevatorConstants`, `PivotConstants`, `WristConstants`

Each has the same six families of fields, by convention:

1. **Hardware IDs.** `LEAD_ID`, `FOLLOWER_ID`, `ENCODER_ID`, DIO ports for limit switches and beam-breaks. ServoHub channels for the pivot ratchets.
2. **Hardware configs.** `CURRENT_LIMIT`, `MOTOR_INVERT`, `NEUTRAL_MODE`, `SENSOR_TO_DEGREE_RATIO` / `ROTOR_TO_INCH_RATIO`, `ENCODER_INVERT`, `ENCODER_ABSOLUTE_OFFSET`, `RATCHET_ON/OFF` pulse widths.
3. **Geometry.** `AXIS_POSITION`, `MASS_LBS`, `COM_TO_STAGE_*`, `STAGE_*_LIMIT`, `ZERO_UPRIGHT_COM`, `AXIS_TO_ZERO_COM` — center-of-mass and pivot-arm geometry used by [`GravityGainsCalculator`](../util/OTHER.md).
4. **Control gains.** `V`, `A`, `P`, `I`, `D` (PID + feedforward), `EXPO_V`, `EXPO_A` (Motion Magic Expo profile). `EXPO_V_SLOW` for the climb-only slow profile on the pivot. `G` — the gravity feedforward function (a `DoubleUnaryOperator` or `BiFunction<Double, Double, Double>` depending on which joints' positions drive the load).
5. **Tolerances.** `SAFE_TOLERANCE` (loose; used by `nearTarget`), `AT_TARGET_TOLERANCE` (tight; used by `isAtTarget`).
6. **State presets.** One field per `State` enum constant — the target position (inches or degrees) that the enum value carries. For the wrist, additionally `UPPER_LIMIT` and `LOWER_LIMIT` are *functions* (`BiFunction<Double, Double, Double>` and `Function<Double, Double>`) of the other joints' positions; this is the dynamic-limit machinery documented in [`WRIST.md`](../subsystems/WRIST.md).

### `IntakeConstants`

`MOTOR_ID`, `SENSOR_ID` (Canandcolor), `BEAMBREAK_DIO_PORT`, `CURRENT_LIMIT`, `MOTOR_INVERT`, `NEUTRAL_MODE`, and the operative `DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD` — the floor of the Canandcolor proximity reading below which a coral is considered "near."

### `SwerveConstants`

The largest single config class. Key fields:

- `PATH_TRANSLATION_CONTROLLER_P/D`, `PATH_ROTATION_CONTROLLER_P` — gains for PathPlanner's `PPHolonomicDriveController` configured in `Swerve` (also used by `FollowPathRequiringAlgaeCommand`).
- `ANGULAR_POSITION_P/D` — yaw PID gains used by `DriveCommand`, `DirectMoveToPoseCommand`, `PathfindToPoseAvoidingReefCommand`, `SearchForObjectCommand`.
- `ANGULAR_OBJECT_DETECTION_P/D` — yaw PID gains for `DriveCommand`'s `OBJECT_HEADING` mode (different scale than tag-yaw error).
- `ANGULAR_MINIMUM_ANGLE`, `ANGULAR_MAXIMUM_ANGLE` — typically ±180°; passed to `enableContinuousInput` so the PID wraps correctly.
- `SLIP_CURRENT` — the stator-current threshold above which a drive motor is considered "slipping/stuck"; consumed by `Swerve.moduleStuck` triggers.
- `AUDIO_CONFIGS` — shared `AudioConfigs` applied to every motor for Orchestra (see `Song`).
- `SWERVE_DRIVETRAIN_CONSTANTS`, `FRONT_LEFT`, `FRONT_RIGHT`, `BACK_LEFT`, `BACK_RIGHT` — CTRE-generated module constants from Tuner X. These are nested record-like objects; the variant classes (`CompConstants.SwerveConstants.FrontLeft.FRONT_LEFT`, etc.) supply the per-robot versions.

## Functional fields

The most "advanced" constants are functions rather than scalars:

- `MAX_CONTROLLED_VEL.apply(elevatorHeight)` — `DoubleUnaryOperator`. Decays with elevator extension.
- `MAX_CONTROLLED_ANGULAR_VEL.apply(elevatorHeight)` — same shape.
- `ElevatorConstants.G.apply(pivotAngle)` — gravity feedforward for the elevator.
- `PivotConstants.G.apply(pivotAngle, wristAngle, elevatorHeight)` — `TriFunction`-style closure produced by [`GravityGainsCalculator`](../util/OTHER.md).
- `WristConstants.G.apply(wristAngle, pivotAngle)` — `BiFunction`.
- `WristConstants.UPPER_LIMIT.apply(elevatorHeight)` and `LOWER_LIMIT.apply(elevatorHeight, pivotAngle)` — dynamic soft limits.
- `VISION_STD_DEV_FUNCTION` and `VISION_STD_DEV_MULTITAG_FUNCTION` — distance-keyed covariance matrices.

These are stored as `final` lambdas/closures inside the variant classes and copied over by `RobotIdentity.setDefaultConstants()` / per-variant setters. The math they encode is in the variant source; this doc summarizes their *roles*.

## Robot Variants

The variant classes under [`constants/variants/`](../../src/main/java/io/github/frc461/rowdy25/constants/variants/) follow the dispatch table in [`ROBOT_IDENTITY.md`](ROBOT_IDENTITY.md):

- **`DefaultConstants`** — baseline; used by ALPHA and as the foundation for every other variant.
- **`CompConstants`** — competition robot (ROWDY) overrides. The biggest delta from defaults: swerve module geometry, every camera mount transform, the elevator/pivot/wrist mechanical constants, and most PID/feedforward gains.
- **`SimConstants`** — overrides only the angular controllers' D gains because the sim model lacks the real inertia.
- **`TestConstants`** — overrides the swerve and the two front cameras; only what's needed to drive the test bed safely.

## Usage

Access values through nested-class field paths, e.g., `Constants.ElevatorConstants.STOW` or `Constants.SwerveConstants.SLIP_CURRENT.in(Amps)`. Code should *never* hard-code numeric tuning values inline — add them to `DefaultConstants` and the relevant variants instead.

## Implementation note on tuning

The constants here encode a huge amount of empirically-determined tuning information (PID gains, profile coefficients, σ functions, gravity-feedforward maps, mount transforms, encoder offsets). The math behind each constant's *role* is documented elsewhere in this doc tree; the *numeric values* are the result of practice-field iteration. To change a value, edit the appropriate variant class and re-flash — there is no SmartDashboard hot-tune path for most fields, by design.

## See Also

- [`RobotIdentity`](ROBOT_IDENTITY.md) — Variant dispatch and the two-pass assignment pattern.
- [`RobotPoses`](ROBOT_POSES.md) — Field-relative pose tables that consume the geometry constants here.
- Subsystem docs ([`ELEVATOR`](../subsystems/ELEVATOR.md), [`PIVOT`](../subsystems/PIVOT.md), [`WRIST`](../subsystems/WRIST.md), [`INTAKE`](../subsystems/INTAKE.md), [`DRIVETRAIN`](../subsystems/DRIVETRAIN.md), [`LOCALIZER`](../subsystems/LOCALIZER.md)) — Consumers of each nested constants class.
- [`GravityGainsCalculator`](../util/OTHER.md) — Produces the `G` functions.
