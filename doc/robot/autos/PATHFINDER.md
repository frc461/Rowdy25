# Pathfinder Utility

[Pathfinder](../../src/main/java/io/github/frc461/rowdy25/autos/Pathfinder.java) is a standalone, `final` utility class that wraps PathPlanner's `AutoBuilder.pathfindToPose` with the geometric helpers needed to compute "approach" poses around reef branches, coral stations, algae faces, and other field landmarks.

> **Status:** As of the current revision, `Pathfinder` is **not referenced by any production class.** Both `AutoManager` and the Swerve auto routines obtain their pathfinding commands through `Swerve.pathFindTo<X>(...)` helpers, which internally build [`PathfindToPoseAvoidingReefCommand`](../commands/DRIVE.md) instances rather than calling PathPlanner's `AutoBuilder` directly. `Pathfinder` is retained as a reference / experimentation surface and is exercised only by its own `main(...)` console-test harness. It also remains the easiest way to invoke raw `AutoBuilder.pathfindToPose` if a future routine ever needs the PathPlanner-side planner without the reef-avoidance wrapper.

## What `Pathfinder` Actually Does

`Pathfinder` is purely a *computation* layer. It does not own a subsystem, does not require anything, and never directly drives motors — every public method either returns a `Pose2d` or returns a `Command` produced by `AutoBuilder.pathfindToPose(...)`.

### Building blocks

- `pathFindToPose(Pose2d, double)` / `pathFindToPose(Pose2d)` — Private. Delegates to `AutoBuilder.pathfindToPose` with `Constants.AutoConstants.PATH_CONSTRAINTS` and an optional non-zero goal end velocity.
- `calculateClosePose(Pose2d target, double distance)` — Returns the pose obtained by stepping `distance` meters *backward* along the target's own heading (i.e., applies a `Transform2d((-distance, 0), 0)` to the target). Useful when you want the robot to stop a fixed distance short of a target it must approach head-on.
- `calculateClosePose(Pose2d target, double distance, Rotation2d heading)` — Generalized version that offsets along an arbitrary direction relative to the target's frame.
- `calculateClosePoseWithAngleScopeAndRadius(current, target, lowerθ, upperθ, distance)` — Private. Given current and target translations and an allowed angular window `[lowerθ, upperθ]` around the target, returns the constrained approach pose on the circle of radius `distance` around the target:
  - Let `φ = atan2(current − target)`. If `RotationUtil.inBetween(φ, lowerθ, upperθ)`, return the pose at angle `φ` on the circle, facing the target (`φ + π`).
  - Otherwise snap to whichever of the two boundary poses (at `lowerθ` or `upperθ`) is closer to the current pose via `Pose2d.nearest(...)`.

### Pathfinding helpers

These all return a `Command`:

- `pathFindToClosePose(targetPose, distance, goalEndVelocity)` — Pathfind to the "behind the target" close pose.
- `pathFindToClosePose(currentPose, targetPose, distance)` — Pathfind to any point on the radius-`distance` circle around `targetPose`. Greedy: minimizes traveled distance (no angular restriction).
- `pathFindToClosePose(currentPose, targetPose, lowerθ, upperθ, distance[, goalEndVelocity])` — Same idea, but constrains the approach angle to a window. Early-exits with `Commands.none()` if the robot is already within `distance` of the target.
- `pathFindToNearestAlgaeScoringLocation(currentPose)` — Resolves the nearest algae scoring tag via `FieldUtil.AlgaeScoring`, flips its rotation by 180° (so the robot ends up facing it), and dispatches `pathFindToClosePose` with `Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE` as the standoff distance.
- `pathFindToNearestCoralScoringLocation(mode, currentPose)` — Resolves the nearest reef branch pose for the given `RobotScoringSetting` (e.g., `AT_BRANCH`, `JUST_BEFORE_BRANCH`) and dispatches `pathFindToClosePose` with the standoff distance.

### `main(...)` harness

The `main` method is a desktop entry point used to print interpolated coral-station poses and verify the velocity-profile multiplier `0.04 · 5 · ln(5e^{2.5} + 5 − 1)`. It is **not** executed on the robot.

## Why Production Code Uses `PathfindToPoseAvoidingReefCommand` Instead

`AutoBuilder.pathfindToPose` (and therefore `Pathfinder`) relies on PathPlanner's `LocalADStar` planner with a precomputed reef obstacle inflation. In practice the Rowdy25 routines need (a) tight, deterministic timing inside the 15-second autonomous, (b) a velocity profile that scales with elevator height for tip stability, and (c) a continuously-updated tangent-around-the-reef waypoint when the robot starts on the wrong side. These requirements are met by [`PathfindToPoseAvoidingReefCommand`](../commands/DRIVE.md), which implements the avoidance and velocity profile directly. The Swerve helpers (`Swerve.pathFindToNearestLeftBranch(...)`, `pathFindToScoringLocation(...)`, `pathFindToLeftCoralStation(...)`, `pathFindToNet(...)`, etc.) all instantiate this command, and `AutoManager.generateAutoEventLooper(...)` composes them into the autonomous event loop.

## See Also

- [`PathfindToPoseAvoidingReefCommand`](../commands/DRIVE.md) — The reef-aware command used by actual auto routines and teleop "smart bumper" bindings.
- [`AutoManager`](AUTO_MANAGER.md) — Where the Swerve helpers (not `Pathfinder`) are stitched together.
- [`FieldUtil`](../util/OTHER.md) — Source of landmark math and nearest-target queries.
- [`RobotPoses`](../constants/ROBOT_POSES.md) — Per-landmark target poses.
- [`Localizer`](../subsystems/LOCALIZER.md) — Provides `getStrategyPose()` used as the input "current pose" for the geometric helpers.
