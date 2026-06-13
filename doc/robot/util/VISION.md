# Vision Utilities

The classes in [`util/vision/`](../../src/main/java/io/github/frc461/rowdy25/util/vision/) wrap each camera / SLAM API used by Rowdy25 in a uniform interface that the [Localizer](../subsystems/LOCALIZER.md) can consume. Each utility's job is to convert raw camera observations into a *field-frame* `Pose2d` (with associated timestamp and standard deviation) that can be fed into WPILib's `SwerveDrivePoseEstimator` (`addVisionMeasurement(pose, timestamp, stdDev)`).

This page focuses on the *math* — how each camera pipeline derives a robot pose, and when the single-camera vs. multi-camera variants are used.

## Background: How AprilTag Localization Works

An AprilTag detection gives the camera the tag's 4 corners in image space. The standard approach is to feed those 4 known 3D ↔ 2D point correspondences into Perspective-n-Point (PnP) — typically OpenCV's `SOLVEPNP_IPPE_SQUARE` solver for square planar targets — which yields a `Transform3d cameraToTag`. Given the tag's known field pose $T^F_{\text{tag}}$ and the camera's mount transform $T^R_{\text{cam}}$:

$$
T^F_{\text{robot}} = T^F_{\text{tag}} \cdot (T^{\text{cam}}_{\text{tag}})^{-1} \cdot (T^R_{\text{cam}})^{-1}.
$$

There are two practical complications:

1. **Single-tag ambiguity.** With one planar tag, IPPE returns *two* solutions related by a reflection through the tag plane. Picking the wrong one flips the robot pose to a mirrored location. PhotonVision exposes both as `getBestCameraToTarget()` and `getAlternateCameraToTarget()` along with a numeric `getPoseAmbiguity()` ∈ [0, 1].
2. **Noise scales with range.** Pixel error projects into world error roughly linearly with tag distance, so observations of distant tags must be down-weighted in the Kalman fusion.

The two utility families below handle these complications differently.

## LimelightUtil — Single-Camera, Fused-Tag Localization

[LimelightUtil](../../src/main/java/io/github/frc461/rowdy25/util/vision/LimelightUtil.java) wraps the Limelight NetworkTables API. The pose comes from one of two NT entries:

### MegaTag1 — `botpose_wpiblue`

`getMegaTagOneValues()` reads the Limelight's internal MegaTag1 solve, which stacks **all currently-visible tags** into a single PnP problem. With $n$ tags visible, the optimizer has $4n$ point correspondences but only the same 6 robot-pose unknowns, so the system is heavily overdetermined for $n \ge 2$ — this collapses the single-tag mirror ambiguity. With $n = 1$, however, MegaTag1 is **just as ambiguous as raw PnP**, so the Localizer admits it only when `LimelightUtil.isMultiTag() && LimelightUtil.isTagClear()`:

```java
if (LimelightUtil.isMultiTag() && LimelightUtil.isTagClear()) {
    Pose2d megaTagPose = LimelightUtil.getMegaTagOnePose();
    poseEstimator.addVisionMeasurement(megaTagPose, timestamp, stdDev);
}
```

`isTagClear()` is a simple range gate — it rejects tags farther than `BW_MAX_TAG_CLEAR_DIST`.

### MegaTag2 — `botpose_orb_wpiblue`

MegaTag2 fixes the yaw prior to a value supplied by the robot (`setRobotOrientation(yaw)`), then re-solves PnP with that yaw locked. Mathematically, this reduces the 6-DOF unknown to a 3-DOF translation + roll/pitch problem, which has a unique solution from a *single* tag. Trade-off: the resulting translation is only as good as the supplied yaw, so MegaTag2 is excellent for high-yaw-rate maneuvers but only if the gyro is well-calibrated. Rowdy25 publishes MegaTag2 to telemetry but currently feeds only MegaTag1 into the estimator (MegaTag2 is the "backup" pose source if MegaTag1 yaws are too noisy).

## PhotonUtil.BW — Multi-Camera Localization

[PhotonUtil](../../src/main/java/io/github/frc461/rowdy25/util/vision/PhotonUtil.java) drives three black-and-white global-shutter cameras (`TOP_RIGHT`, `TOP_LEFT`, `BACK`), each with its own `Transform3d robotToCameraOffset`. Every loop, `Localizer.updatePhotonPoseEstimation()` iterates all three cameras and merges their measurements into the same `SwerveDrivePoseEstimator`:

```java
PhotonUtil.updateResults(poseEstimator.getEstimatedPosition().getRotation());
for (BWCamera camera : BWCamera.values()) {
    PhotonUtil.BW.getBestTagPose(camera).ifPresent(
        poseEstimate -> poseEstimator.addVisionMeasurement(
            poseEstimate.estimatedPose().toPose2d(),
            poseEstimate.timestampSeconds(),
            poseEstimate.standardDeviations()
        )
    );
}
```

`getBestTagPose(camera)` dispatches between two branches:

### `getMultiTagPose(camera)` — Multi-tag PnP

When `isMultiTag(camera)` is true (camera sees ≥ 2 tags), PhotonVision exposes a precomputed `MultiTargetPNPResult.estimatedPose.best`, which is a `Transform3d fieldToCamera` obtained by stacking all tag detections into one PnP solve (the same idea as MegaTag1, but per-camera). The robot pose follows from:

$$
T^F_{\text{robot}} = T^F_{\text{origin}} \cdot T^F_{\text{camera}} \cdot (T^R_{\text{cam}})^{-1}.
$$

In code:

```java
Pose3d bestPose = new Pose3d()
        .plus(multiTargetPNPResult.estimatedPose.best)   // field-to-camera
        .relativeTo(FieldUtil.ORIGIN)
        .plus(camera.robotToCameraOffset.inverse());     // camera-to-robot
```

The standard deviation is the closest tag's distance fed through `VISION_STD_DEV_MULTITAG_FUNCTION` — the multi-tag function uses smaller σ than the single-tag function, reflecting higher trust.

### `getSingleTagPose(camera, currentPose)` — Single-tag PnP with disambiguation

When only one tag is visible, PhotonVision returns both PnP solutions. The disambiguation logic:

1. If `poseAmbiguity < 0.15`, the solver is confident — accept the *best* solution outright.
2. Else if `poseAmbiguity < 0.4`, compare the rotation of each candidate against the **current pose's rotation** (the gyro-anchored prior held in `SwerveDrivePoseEstimator`) and accept whichever is closer. This is the textbook "use a prior to break PnP mirror ambiguity" trick.
3. Else reject the measurement.

Finally `FieldUtil.isInField(...)` discards any candidate that falls outside the field rectangle. The measurement's σ is `VISION_STD_DEV_FUNCTION(distance)`.

There is also a *private* `getSingleTagPose(camera)` variant — currently unused by `getBestTagPose` — that side-steps PnP disambiguation entirely by reconstructing the camera-to-tag vector from the tag's pixel yaw/pitch (small-angle geometry) and then using the **historical yaw buffer** `headingBuffer` to anchor the rotation. This is the same trick MegaTag2 uses, but reconstructed in client code. It exists because, in practice, a known-good yaw makes single-tag localization more accurate than disambiguated PnP — at the cost of trusting the gyro.

### Why three cameras?

With three cameras the Localizer effectively gets **three independent observations per loop**. Each is `addVisionMeasurement(...)`-ed into the same Kalman fusion, so the estimator sees them as three weighted measurements at slightly-different timestamps. The redundancy:

- Recovers from individual occlusions (e.g., a coral in front of one lens).
- Provides geometric coverage in front (TOP_LEFT / TOP_RIGHT) and behind (BACK) the robot.
- Lets multi-tag PnP fire from any camera independently — if even one camera sees ≥ 2 tags, that observation will dominate the others through its smaller σ.

## QuestNavUtil — Backup SLAM Localization

[QuestNavUtil](../../src/main/java/io/github/frc461/rowdy25/util/vision/QuestNavUtil.java) reads pose telemetry from a Meta Quest headset running QuestNav, which performs visual-inertial SLAM independently of AprilTags. The headset publishes its own pose in its own frame:

- `getRawX/Y/Z`, `getRawYaw/Pitch/Roll` — Raw Quest pose with axis conventions remapped to WPILib's field frame.
- `getRobotPose()` — Applies the inverse of `QuestNavUtil.robotToCameraOffset` to convert headset pose → robot pose.
- `setQuestPose(robotPose)` / `completeQuestPose()` — Two-way handshake with the headset via `questMiso`/`questMosi` request codes to recalibrate the Quest origin when a tag-based pose is known to be accurate (this is how `Localizer.calibrateQuestNav(...)` re-anchors the SLAM frame on confirmed AprilTag fixes).

QuestNav is exposed in the [Localizer](../subsystems/LOCALIZER.md) as a switchable `LocalizationStrategy.QUEST_NAV`; `Localizer.getStrategyPose()` returns either the QuestNav pose or the AprilTag-fused estimator pose depending on the SmartDashboard chooser. This means QuestNav is *not* fused into the pose estimator — it replaces it. The point of the switch is to keep driving when tags are obscured (lighting, opponents standing in front of the reef, etc.).

## Driver Trust Override

`Localizer.toggleTrustCameras()` (bound to a driver button) flips the `trustCameras` flag. When false:

- `currentRobotScoringSetting` snaps to `AT_BRANCH` regardless of vision (i.e., the operator commits to driving "by feel").
- `isAgainstReefWall()` / `isAgainstCoralStation()` return `true` unconditionally so superstructure state machines no longer wait on vision confirmations.

This is an emergency fallback for matches where one or more cameras are physically damaged or NetworkTables traffic stalls.

## Standard Deviation Functions

All measurements ultimately reach the same Kalman fusion:

| Source | σ function | Notes |
| --- | --- | --- |
| Limelight MegaTag1 | constant (Limelight-side) | only admitted when `isMultiTag && isTagClear` |
| Photon multi-tag | `VISION_STD_DEV_MULTITAG_FUNCTION(bestTagDist)` | smallest σ; dominates fusion |
| Photon single-tag | `VISION_STD_DEV_FUNCTION(distance)` | larger σ; ambiguity-checked |

Because the pose estimator is a weighted-least-squares fuse (constant-velocity Kalman with the odometry sample as prior), smaller σ → stronger pull toward that measurement. Distance-dependent σ is what makes the system "trust the nearest tag" without explicit branching in the Localizer.

## See Also

- [Localizer](../subsystems/LOCALIZER.md) — Consumer of all vision utilities and host of `SwerveDrivePoseEstimator`.
- [`VisionConstants`](../constants/CONSTANTS.md) — Camera mount transforms, distance gates, and σ-function coefficients.
- [`EstimatedRobotPose`](OTHER.md) — Record used to carry `(pose, timestamp, targets, stdDev)` between PhotonUtil and Localizer.
