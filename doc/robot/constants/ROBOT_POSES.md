# RobotPoses Class

[`RobotPoses`](../../src/main/java/io/github/frc461/rowdy25/constants/RobotPoses.java) computes the target robot `Pose2d`s for every field landmark relevant to the 2025 Reefscape game. Where [`FieldUtil`](../util/OTHER.md) describes the *field* (where AprilTags, branches, sides, etc. live), `RobotPoses` describes where the *robot chassis* should sit relative to those features so the gripper can score / intake.

Every method takes a `currentPose` (or a `ScoringLocation` / `Side`) and returns one or more `Pose2d`s. All poses are in blue-alliance field coordinates; PathPlanner's `FlippingUtil` (and `Constants.ALLIANCE_SUPPLIER`) is the alliance-flip mechanism, applied at the point of use rather than baked into the lookup.

## Nested Namespaces

### `RobotPoses.Reef`

#### `RobotScoringSetting` enum

Selects which sub-pose is used for L2/L3/L4 scoring:

- **`L1`** — forward offset + lateral branch offset for the trough.
- **`L2`** — slightly looser offset for L2.
- **`AT_BRANCH`** — flush against the reef face.
- **`ONE_CORAL_FROM_BRANCH`** — one coral-width back; leaves room for an "intake-out" backwards outtake.

[`Localizer`](../subsystems/LOCALIZER.md) chooses this setting based on `trustCameras` and the operator level overrides; downstream callers read it via `localizer.currentRobotScoringSetting`.

#### Branch poses

- `getRobotPoseAtBranch(mode, location)` — Look up the robot pose at the named `ScoringLocation` (A–L) under `mode`. Implemented as nested `switch` blocks that resolve to AprilTag-anchored transforms.
- `getRobotPoseNearBranch(mode, location)` — Same idea, but offset *back* by the standoff distance. Consumed by `PathfindToPoseAvoidingReefCommand` for the standoff before the final direct-move.
- `getNearestRobotPoseAtBranch(mode, currentPose)` — `currentPose.nearest(getRobotPosesAtBranches(mode))`. A linear-scan over the 12 precomputed branch poses.
- `getRobotPosesAtBranches(mode)` — Lists all 12 precomputed branch poses for the mode.
- `getNearestRobotPosesAtBranchPair(mode, currentPose)` — Returns the *two nearest* branch poses (the L/R pair on whichever reef face the robot is closest to). Used by the driver's LB/RB bindings, which pre-resolve to the left or right branch of the nearest face.
- `getNearestRobotPosesNearBranchPair(mode, currentPose)` — Same as above but returns the *near* poses (standoff before the final approach).

#### Algae poses

- `getRobotPoseAtAlgaeReef(side)` — Per-side flush-against-the-face algae removal pose. The geometry differs by 2"/–5" forward depending on whether `algaeIsHigh(side)` and whether the robot approaches face-on or backward.
- `getNearestRobotPoseAtAlgaeReef(currentPose, algaeIsHigh)` — Picks the nearest face's algae pose.
- `getRobotPoseNearReef(side)` / `getNearestRobotPoseNearReef(algaeIsHigh, currentPose)` — Standoff variants.

#### `sameSide(currentPose, targetPose)` — The reef-sextant predicate

The heart of the reef avoidance check. `PathfindToPoseAvoidingReefCommand`'s branch-1 ("no obstacle in the way") fires when this returns true. The algorithm:

1. Compute the four *bumper corners* of the robot at `currentPose` (the chassis rectangle extended by the bumper width / length).
2. For each of the 6 reef sides, gather the angles and distances from the chosen reef-vertex anchor to each of the 4 corners.
3. Determine which sextant the target lies in.
4. Test whether *every* robot corner is on the same side of the dividing line as the target. If yes, the chassis can drive a straight line without crossing a reef face.

This is more conservative than a center-point check because it accounts for the chassis's actual footprint — the robot can be *centered* in a same-side sextant but have one bumper poking into the neighboring sextant, and the predicate catches that.

### `RobotPoses.CoralStation`

- `getRobotPosesAtEachCoralStation()` — Pair of `Pose2d`s computed by extending each coral-station AprilTag forward by half the chassis length. Used by `Swerve.pathFindToLeftCoralStation` / `pathFindToRightCoralStation` and by `Localizer.updateRobotUtilityPoses`.
- `getNearestRobotPoseAtCoralStation(currentPose)` — `currentPose.nearest(...)` over the pair.

### `RobotPoses.AlgaeScoring`

- `getRobotPoseAtNetCenter(currentPose)` — The robot pose that lines the gripper up with the net center. Offset by half the chassis length, rotated 180° because the gripper points *backwards* when scoring algae into the net.
- `getInnermostRobotPoseAtNet(currentPose)`, `getOutermostRobotPoseAtNet(currentPose)` — Bounds for the randomized net pose used by `Swerve.pathFindToNet(randomized=true)`. The X coordinate of the chosen scoring pose is uniformly random between these two.
- `getCurrentAllianceSideRobotPoseAtProcessor(currentPose)` — The robot pose at the processor opening, offset by half the chassis length + 0.5 m (so the gripper actually clears the opening's lip). The "current alliance side" qualifier is because the processor is on the driver-station-side wall, so the offset must be computed alliance-aware.

## Why pre-computed tables?

Every `RobotPoses.*` lookup runs every loop in `Localizer.updateRobotUtilityPoses()`. Recomputing the pose math at 50 Hz × 30+ lookups × per-tag transform stacking is non-trivial CPU. By precomputing the constant offsets at startup and only performing the alliance-flip and the geometric search at runtime, the localizer can re-resolve every nearest landmark every tick without breaking the loop budget.

## Alliance handling

All poses are stored in blue-alliance field coordinates. The flip is applied at the point of consumption:

- `FieldUtil.AprilTag.getTag(id).pose2d` returns the blue pose.
- `Constants.ALLIANCE_SUPPLIER.get() == Red` is the flip condition.
- `FlippingUtil.flipFieldPose(pose)` is the flip operation (from PathPlanner).

`AutoManager.getStartingPose(...)` and the `Swerve.pathFindTo*` family demonstrate the pattern.

## See Also

- [`FieldUtil`](../util/OTHER.md) — Source of the AprilTag poses, `Reef`, `Side`, `ScoringLocation`, `Level`, `CoralStation`, and `AlgaeScoring` namespaces this class composes.
- [`Localizer.updateRobotUtilityPoses`](../subsystems/LOCALIZER.md) — Per-tick consumer of every method here.
- [`PathfindToPoseAvoidingReefCommand`](../commands/DRIVE.md) — Uses `sameSide` for the no-obstacle fast path.
- [`AutoManager`](../autos/AUTO_MANAGER.md) — Uses the discrete branch/algae pose tables to compile the autonomous chain.
