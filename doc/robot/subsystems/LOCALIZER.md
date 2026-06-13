# Localizer Subsystem

[`Localizer`](../../src/main/java/io/github/frc461/rowdy25/subsystems/localizer/Localizer.java) maintains the robot's field-relative pose by fusing wheel odometry with vision corrections, optionally substitutes a Meta Quest SLAM track, and *also* serves as the single source of truth for every "nearest landmark" pose that drive commands and `RobotStates` consume. It is **not** a WPILib `Subsystem` — it is owned by `Swerve` and called from `Swerve.periodic()`.

## Construction

```java
poseEstimator = new SwerveDrivePoseEstimator(
    swerve.getKinematics(),
    swerve.getState().RawHeading,
    swerve.getState().ModulePositions,
    swerve.getState().Pose,
    Constants.VisionConstants.ODOM_STD_DEV,
    Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION.apply(1.0)
);
```

`SwerveDrivePoseEstimator` is an unscented-Kalman-like fusion that maintains a sliding window of timestamped odometry samples and accepts `addVisionMeasurement(pose, timestamp, stdDev)` calls. The construction-time vision σ is the multi-tag function evaluated at 1.0 m — a "typical" value; per-measurement σ overrides this at runtime in `updatePhotonPoseEstimation` and `updateLimelightPoseEstimation`.

After construction the SmartDashboard chooser is wired (`POSE_ESTIMATOR` default, `QUEST_NAV` alternate), the QuestNav origin is calibrated to the estimator's initial pose, and Limelight's `robotToCamera` offset is pushed to the Limelight via NetworkTables.

## `LocalizationStrategy` chooser

```java
public Pose2d getStrategyPose() {
    return strategy == LocalizationStrategy.QUEST_NAV ? getQuestPose() : getEstimatedPose();
}
```

This is the single accessor that downstream callers use ("the pose"). Switching strategies is operator-driven through SmartDashboard; `setLocalizationStrategyFromChooser()` is called every `periodic()` to honor a mid-match change. The QuestNav branch entirely **replaces** the estimator (it is not fused), because doing so would require a full UKF re-architecture — the current design is "use whichever source is more trustworthy right now."

## Pose Fusion Pipeline

`periodic()` runs the following in order:

1. `localizationTelemetry.publishValues()` — telemetry first so consumers see the *previous* tick's converged data while this tick computes.
2. `poseEstimator.update(rawHeading, modulePositions)` — pure odometry integration.
3. `updatePhotonPoseEstimation()` — for each `BWCamera`, if `isTagClear(camera)`, call `getBestTagPose(camera)` and `addVisionMeasurement(...)`. See [`VISION.md`](../util/VISION.md) for what `getBestTagPose` does internally.
4. `setLocalizationStrategyFromChooser()` — read chooser, mutate `strategy` if changed.
5. `updateRobotUtilityPoses()` — recompute every `nearest*` pose against the just-updated `getStrategyPose()`.

Notably absent from `periodic()`: `updateLimelightPoseEstimation()` and `updateQuestNavPose()`. They are *defined* on the class (and the Limelight method gates on `LimelightUtil.isMultiTag() && isTagClear()`, applies the multi-tag σ scaled by nearest tag distance, and subtracts Limelight pipeline latency for the timestamp) but are not currently called every tick — the PhotonVision pipeline is the production fusion source. `updateQuestNavPose()` exists for the QuestNav recalibration handshake; `forceUpdateQuestNavPose()` is exposed for triggers that need to re-anchor the SLAM origin manually.

## `trustCameras` and the Scoring Setting

`trustCameras` is a public boolean flipped by `toggleTrustCameras()` (bound to the driver's A-button chord; see [`ROBOT_CONTAINER.md`](../ROBOT_CONTAINER.md)). It interacts with the scoring-setting state machine:

```java
private void updateCoralScoringMode() {
    if (!trustCameras)                      currentRobotScoringSetting = AT_BRANCH;
    else if (l1RobotScoringSettingOverride) currentRobotScoringSetting = L1;
    else if (l2RobotScoringSettingOverride) currentRobotScoringSetting = L2;
    else                                    currentRobotScoringSetting = ONE_CORAL_FROM_BRANCH;
}
```

The setting changes the `RobotPoses.Reef.getRobotPoseAtBranch / getRobotPoseNearBranch` table lookups — a `ONE_CORAL_FROM_BRANCH` mode produces an approach pose that stops one coral width short of the reef (room for a backwards "intake-out" outtake), `AT_BRANCH` produces a flush pose. So flipping `trustCameras` off implicitly snaps the robot to the safer flush approach because mid-air vision corrections become untrustworthy.

`isAgainstReefWall()` and `isAgainstCoralStation()` short-circuit to `true` when `!trustCameras` so superstructure state machines stop waiting on vision-derived "docked" predicates.

## Pose Predicates (consumed by `RobotStates`)

Three tolerance bands query `getDistanceToActionLocation(state)` against the per-state nearest pose:

| Predicate | Tolerance | Used for |
| --- | --- | --- |
| `atScoringLocation(state)` | `TRANSLATION_TOLERANCE_TO_ACCEPT` (tightest) | Fires the actual outtake. |
| `nearStateLocation(state)` | `TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE` (mid) | Transitions from `PathfindToPoseAvoidingReefCommand` to `DirectMoveToPoseCommand`. |
| `atTransitionStateLocation(state, auto)` | `TRANSLATION_TOLERANCE_TO_TRANSITION` (teleop) / `_AUTO` (looser) | Pre-arm superstructure state so mechanisms are in place by the time the chassis docks. |

`getRobotRelativeVectorToActionLocation(state)` returns the *robot-frame* offset vector to the action pose, used by the wall-contact predicates above (X component is "in front of / behind the robot" — within 1 inch is "wall contact").

## `updateRobotUtilityPoses()` — the per-tick pose recomputation

This method is the reason every `pathFindTo*` helper just reads `localizer.nearest*` fields:

- **Coral detection.** `PhotonUtil.Color.getRobotToBestObject(CORAL)` returns the robot-relative vector; the localizer projects it into field frame and offsets by half the robot length + 12 inches to get an approach pose 12 inches *past* the detected coral (so the intake passes over it). The 12-inch offset is the empirically tuned grab distance.
- **Reef branches.** Single nearest (`nearestRobotPoseAtBranch`), the pair of two nearest left+right (`nearestRobotPosesAtBranchPair`), and the *near*-branch variants (`nearestRobotPosesNearBranchPair`) used as PathfindToPoseAvoidingReefCommand targets. The "near" poses are the standoff before the direct-move final approach.
- **Both-reefs reef tag.** `nearestReefTagPoseBothReefs` uses the `true` "both reefs" flag — i.e., considers the opposite-alliance reef as well — for heading lock during cross-field traversal.
- **Net center.** Plain `getRobotPoseAtNetCenter`. The randomized variant is *not* recomputed here — it's recomputed only when `randomizeNetScoringPose()` is called from `pathFindToNet(true)`. This way the operator's commanded random pose stays stable for the duration of the pathfind.
- **Processor.** Alliance-side aware: `getCurrentAllianceSideRobotPoseAtProcessor`.
- **Coral station.** Picks the nearer of the two alliance-flipped station-corner interpolations using `Pose2d.nearest(List.of(...))`.
- **Algae reef.** `nearestRobotPoseAtAlgaeReef` (docked) and `nearestRobotPoseNearAlgaeReef` (standoff), with the high/low choice tracked separately as `nearestAlgaeIsHigh`.

Because all of this runs every loop, the operator can press LB/RB at any time and get the most recently nearest pose — there's no "I pressed it too early and the wrong branch was captured" failure mode.

## QuestNav Calibration

`updateQuestNavPose()` (manually invoked when the strategy is QuestNav) implements a guarded re-anchor:

```text
if Limelight reports the nearest tag is "far" (> MIN_TAG_DIST_TO_BE_FAR):
    clear the "calibrated near" flag
if not calibrated near AND
   Limelight tag is clear AND
   any BW Photon tag is clear AND
   chassis is fully stationary (all three velocities exactly zero):
        push current pose estimator pose into Quest origin
        latch "calibrated near"
```

The "fully stationary" requirement means QuestNav can only re-anchor when the chassis is rock-still under a clear tag — exactly when AprilTag-based localization is at its most accurate. The "far → unlatch" rule ensures the *next* close-up encounter retriggers the calibration.

## Pose Reset / Sync

Three reset entry points are exposed:

- `setPoses(pose)` — replaces every pose source (pose estimator, raw swerve, Quest origin) with the given pose. Used by the driver coral-station-reset bindings.
- `setRotations(heading)` — replaces every heading source.
- `syncRotations()` — replaces every heading source with the pose estimator's current heading. Called by `Swerve.periodic()` while disabled to keep gyro and pose-estimator yaw aligned.

## `LocalizerSim`

[`LocalizerSim`](../../src/main/java/io/github/frc461/rowdy25/subsystems/localizer/LocalizerSim.java) is the simulation peer. It updates the vision-simulation scene (PhotonVision `VisionSystemSim`) with the current strategy pose every tick so simulated cameras see field-correct AprilTag projections. Fidelity is "good enough to validate fusion math," not a full optical simulation.

## `LocalizationTelemetry`

[`LocalizationTelemetry`](../../src/main/java/io/github/frc461/rowdy25/subsystems/localizer/LocalizationTelemetry.java) publishes the strategy name, every pose source (estimator, Quest, MegaTag1, MegaTag2, best coral, current temporary target), and a SmartDashboard `Field2d` for visual debugging. The topics are flat NT entries; consult the source for the exact names.

## Implementation note on tuning

`ODOM_STD_DEV`, the per-distance vision σ functions, the QuestNav `MIN_TAG_DIST_TO_BE_FAR`, and the 12-inch best-coral approach offset were tuned against the real robot on the practice field. The math is documented above; the numeric values are empirically derived constants in [`Constants.VisionConstants`](../constants/CONSTANTS.md).

## See Also

- [`VISION.md`](../util/VISION.md) — Per-source pose-estimation math for Limelight, PhotonVision, and QuestNav.
- [`Swerve`](DRIVETRAIN.md) — Owns this Localizer; calls `periodic()` once per loop.
- [`RobotPoses`](../constants/ROBOT_POSES.md) — Source of every `getRobotPoseAt*` / `getNearestRobotPose*` table used by `updateRobotUtilityPoses`.
- [`FieldUtil`](../util/OTHER.md) — Source of `Reef`, `AprilTag`, `AlgaeScoring`, `CoralStation` enums and geometry primitives.
