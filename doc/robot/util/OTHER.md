# Other Utilities

Miscellaneous helper classes covering field math, control primitives, characterization, and custom triggers. All live under [`util/`](../../src/main/java/io/github/frc461/rowdy25/util/).

The vision wrappers (`LimelightUtil`, `PhotonUtil`, `QuestNavUtil`) are documented separately in [`VISION.md`](VISION.md).

## `FieldUtil` — Field geometry and AprilTag tables

[`FieldUtil`](../../src/main/java/io/github/frc461/rowdy25/util/FieldUtil.java) owns the project's view of the field. It loads `AprilTagFields.k2025ReefscapeWelded` once and exposes:

- **`layout2025`** — the WPILib `AprilTagFieldLayout`, public for direct queries.
- **`FIELD_LENGTH`, `FIELD_WIDTH`, `ORIGIN`** — geometry primitives in meters / `Pose3d`.
- **`isInField(Pose3d/Pose2d)`** — bounds check used by `PhotonUtil.BW.getSingleTagPose` to reject impossible PnP solutions.
- **`getAllianceSide(Pose2d)`** — `currentPose.getX() < FIELD_LENGTH / 2 ? Blue : Red`. Used wherever the alliance-side of a pose is needed independent of the driver-station alliance.

### Nested classes

- **`AprilTag`** — enum of all 22 tags (`ID_1` … `ID_22`) plus `INVALID`. Each carries the tag's `pose3d` and `pose2d`. `FILTER` is the subset of reef-side tags PhotonVision is configured to recognize. `getTag(double)` is a lookup-by-ID switch.
- **`TagManager`** — `getPosesToTags()` (pose ↔ tag round-trip map), `getTagLocations2d(List<AprilTag>)` (list-of-poses helper). Used heavily by the `nearest*` methods.
- **`CoralStation`** — alliance-aware accessors for the two station tags. `getNearestCoralStationTagPose(currentPose)` and `getNearestCoralStationTag(currentPose)`.
- **`Reef`** — the project's reef vocabulary:
  - `BLUE_REEF_CENTER`, `RED_REEF_CENTER` — derived as the midpoint of the opposite-face tags' translations.
  - `REEF_APOTHEM` — derived as half the distance between two opposite-face tags. This is the number that appears in the reef-avoidance math (see [`DRIVE.md`](../commands/DRIVE.md) § 1.2).
  - `getNearestReefCenter`, `getAngleFromNearestReefCenter` — the α and α-source for branch 3 of the avoidance algorithm.
  - `Side` enum — six sextants (`AB, CD, EF, GH, IJ, KL`). `getLeftVertexPoseOfNearestReef(currentPose, side)` is the vertex anchor used by `RobotPoses.Reef.sameSide`. `algaeIsHigh(side)` looks up the per-face algae level.
  - `ScoringLocation` enum — 12 branches `A` through `L`.
  - `Level` enum — `L1, L2, L3, L4` with int `.level` values.
  - `getReefTags(bothReefs)`, `getOutsideReefTags()`, `getReefCorners()`, `getReefTagPoses(bothReefs)`, `getNearestReefTagPose(currentPose, bothReefs)`, `getNearestReefTag(currentPose, bothReefs)` — the `bothReefs` flag controls whether the opposite-alliance reef's tags are included (true during cross-field traversal heading lock, false for scoring).
  - `AlgaeLocation` enum + `getAlgaeReefLevelFromTag(tag)` — maps tags to LOW/HIGH algae locations.
- **`AlgaeScoring`** — `ScoringLocation` enum (`NET`, `PROCESSOR`), the net-length and net-safe-half-length constants, and a family of alliance-aware `getCurrentAllianceSideProcessorTagPose`, `getNetTagPoses`, and the nearest-x accessors.

### Why so many `nearest*` methods?

WPILib's `Pose2d.nearest(List<Pose2d>)` does a linear scan with `Translation2d.getDistance`. By precomputing the candidate list inside `FieldUtil` (e.g., "all algae scoring tags for this alliance"), every callsite gets a one-liner with predictable performance. Localizer calls this dozens of times per tick across all the consumers.

## `RotationUtil`

Two functions:

- **`inBetween(angle, lower, upper)`** — wraparound-aware angular containment. Returns true iff `angle` lies in the arc from `lower` to `upper` *going CCW*. If `lower > upper` numerically, the arc wraps through ±180°. This is the predicate used by `Pathfinder.calculateClosePoseWithAngleScopeAndRadius` to decide whether the direct-line approach is within the allowed angular window.
- **`getBound(List<Rotation2d>)`** — returns the (min, max) pair across a list, wraparound-aware.

These look trivial but doing them naively with raw degree comparisons gets the wraparound case wrong; centralizing here means every caller gets the same correct behavior.

## `EquationUtil`

Three primitives in scalar form:

- **`expOutput(error, max, halfway, multiplier)`** — `max / (1 + exp(-multiplier · (error - halfway)))`. The standard logistic / sigmoid curve. Used as a smooth-deceleration profile in `PathfindToPoseAvoidingReefCommand` (with `max=2, halfway=2/7, multiplier=15/2`), as a soft-stop scaler in every subsystem's `move(axis)` method, and as the close-in velocity floor in `DirectMoveToPoseCommand` and `SearchForObjectCommand`.
- **`linearOutput(error, kP, offset)`** — `kP · error + offset`. An affine ramp. Used as the linear-cap velocity term paired with `expOutput`.
- **`polyOutput(error, power, offset)`** — `error^power + offset`. Power-law shape; not currently used in production but available.

The two-arg overloads are convenience defaults (`max=1`, `offset=0`).

## `SysID` — Characterization routines

[`SysID`](../../src/main/java/io/github/frc461/rowdy25/util/SysID.java) wraps WPILib's `SysIdRoutine` for the Swerve drive subsystem. It exposes four routines (translation quasi-static / dynamic, rotation quasi-static / dynamic) and `configureBindings(opXbox)` binds them to operator chord triggers (Back+Start ↔ X/Y) so they can only be triggered intentionally. The standard SysID output is voltage and current logs in DogLog format; the team uses them with WPILib's `sysid` toolchain to derive `kV / kA` for the swerve.

`SysID` is intentionally drivetrain-only; superstructure SysID is done with one-off temporary commands when needed because the gravity feedforward complicates routine reuse.

## `PhoenixProfiledPIDController`

A wrapper around WPILib's `ProfiledPIDController` adapted for Phoenix6 mechanisms. Specifically, it exposes the profile state in a form that maps cleanly to Phoenix6's `MotionMagicVoltage` / `MotionMagicExpoVoltage` setpoints, so closed-loop code can hand off between Java-side and on-motor-controller profiling without losing setpoint continuity. Currently not consumed in the production pipeline — `Elevator/Pivot/Wrist` use Phoenix6's Motion Magic directly — but available as a reference for cases where the profile must live in WPILib (e.g., joint-coordinated motion).

## `ProfiledExpEndController`

A `ProfiledPIDController`-style controller with an *exponential end of motion*. Standard trapezoidal/expo profiles take a long time to truly settle to zero error because the velocity ramp tapers to zero linearly; the exponential-end variant blends in an extra decay term near the setpoint so the position asymptotes smoothly. Used historically in superstructure code; currently retained for future use.

## `GravityGainsCalculator`

The most mathematically involved utility. Given the geometric parameters of a two-stage extension + revolute arm + secondary arm, returns a closure that computes the gravity feedforward voltage as a function of the three joint positions.

- Constructor parameters: pivot axis position, mass distributions, COM offsets, extension limits.
- `calculateGFromPositions(pivotAngle, wristAngle, elevatorPosition)` — returns the voltage equivalent of the gravity torque on the pivot's axis.

This is what produces the `Constants.PivotConstants.G` closure (a `TriFunction<Double, Double, Double, Double>`); the elevator and wrist have simpler one- or two-argument variants. The underlying physics:

$$
\tau = g \cdot \sum_i m_i \cdot r_i(\theta_p, \theta_w, h_e) \cdot \cos(\theta_{\text{effective}, i}),
$$

where each $r_i$ is the effective lever arm of part *i* about the pivot axis, computed from the geometric parameters and the live joint positions. The output is converted to motor voltage via the Phoenix6 sensor-to-mechanism ratio.

The class has a `main(...)` method that prints the computed gravity gain across a grid of joint positions for offline verification.

## `DoubleTrueTrigger`

A static helper:

```java
public static Trigger doubleTrue(BooleanSupplier condition, double timeThreshold) {
    return new Trigger(...);
}
```

Returns a `Trigger` that fires only on the *second* rising edge of `condition` occurring within `timeThreshold` seconds of the first — i.e., a "double-tap" gate. Used for confirmations on potentially-dangerous chord bindings.

## `MultipleChooser`

Extends `SendableChooser` to allow the operator to *order* selections. The standard `SendableChooser` only picks one entry; `MultipleChooser` allows the operator to enter a comma-separated sequence (e.g., `"A4,B3,FRONT,C4"` for an auto routine) and exposes it as a `List<T>`. `AutoManager` consumes it for the scoring/algae sequence.

## `EstimatedRobotPose`

A record-style class that carries `(pose, timestampSeconds, targets, stdDevs)` from `PhotonUtil.BW.getMultiTagPose` / `getSingleTagPose` to `Localizer.addVisionMeasurement(...)`. The four fields are exactly what the `SwerveDrivePoseEstimator.addVisionMeasurement` API needs.

## `MacAddress`

`getMACAddress()` — enumerates `NetworkInterface.getNetworkInterfaces()`, filters out virtual/loopback adapters, and returns the first non-virtual hardware MAC formatted as `XX-XX-XX-XX-XX-XX`. Used by [`RobotIdentity`](../constants/ROBOT_IDENTITY.md) for per-robot constant dispatch. Handles the `SocketException` case by returning empty so the `SIM` fallback fires.

## `Elastic`

A static helper to publish Elastic dashboard notifications:

```java
public static void sendNotification(Notification notification);
```

`Notification` is a Jackson-annotated record carrying `level` (`INFO/WARN/ERROR`), title, description, and width. The class serializes the notification to JSON and publishes it on the NT topic Elastic listens to. The exception-safe `try/catch` means a misconfigured NT instance can't crash the robot code.

## Implementation note on tuning

The numeric parameters scattered through `EquationUtil` callsites (sigmoid steepness, midpoint, linear slope), the SysID voltage rates, and the gravity-gains geometry are all tuned for the specific robot. The math primitives in this directory are general-purpose; the specific instantiations elsewhere in the codebase encode the tuning.

## See Also

- [`Constants`](../constants/CONSTANTS.md) — Stores the `G` closures produced by `GravityGainsCalculator` and the parameters consumed by `EquationUtil`.
- [`RobotPoses`](../constants/ROBOT_POSES.md) — Built on top of `FieldUtil` to produce robot-relative landmark poses.
- [`PathfindToPoseAvoidingReefCommand`](../commands/DRIVE.md) — Uses `FieldUtil.Reef` extensively, plus `EquationUtil` and `RotationUtil`.
- [`RobotContainer.configureButtonBindings`](../ROBOT_CONTAINER.md) — Wires `SysID.configureBindings(opXbox)`.
- [`VISION.md`](VISION.md) — The vision utilities (`LimelightUtil`, `PhotonUtil`, `QuestNavUtil`) — documented separately because their semantics are dense enough to warrant their own page.
